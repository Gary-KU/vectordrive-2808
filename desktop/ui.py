"""Chinese Tkinter CAN console. All transport work belongs to Controller."""
from __future__ import annotations

import csv
import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, ttk
from tkinter import font as tkfont
from tkinter.scrolledtext import ScrolledText

from desktop.controller import Controller
from desktop.current_limits import HOST_CURRENT_LIMIT_A, current_limit_readback, validate_current_target
from desktop.profiles import ProfileError, float32_equal, load_profile, save_profile, validate_parameters
from examples.parameter_catalog import PARAMETER_SPECS, PARAMETERS
from desktop.telemetry import DISPLAY_TIMEOUT_S, can_feedback_error, describe_faults, encoder_readout
from desktop.widgets import COLORS, FONT, EncoderDial, MetricCard, RoundedButton, configure_theme, round_rect


INTERFACES = {
    "PCAN": "pcan",
    "串口 CAN / SLCAN": "slcan",
    "SocketCAN": "socketcan",
    "USB2CAN": "usb2can",
    "CANalyst-II": "canalystii",
    "模拟器（不连接真实电机）": "simulator",
}
CHANNELS = {
    "simulator": "SIM", "pcan": "PCAN_USBBUS1", "slcan": "COM3",
    "socketcan": "can0", "usb2can": "0", "canalystii": "0",
}
_SPECS_BY_ID = {spec.id: spec for spec in PARAMETER_SPECS}
CSV_FIELDS = (
    "timestamp", "interface", "iq_reference_a", "iq_a", "id_a", "velocity_rad_s", "vd_v", "vq_v",
    "angle_rad", "angle_deg", "rpm", "angle_valid", "angle_age_ms",
)


def _number(value, fallback=None):
    try:
        result = float(value)
        return result if math.isfinite(result) else fallback
    except (TypeError, ValueError, OverflowError):
        return fallback


def _display(value, digits=3, suffix=""):
    number = _number(value)
    return "—" if number is None else f"{number:.{digits}f}{suffix}"


class FocDesktopUI:
    """View/controller adapter; no CAN calls or waits run on the Tk thread."""

    def __init__(self, root, controller=None):
        self.root = root
        self.controller = controller if controller is not None else Controller()
        self._snapshot = {}
        self._samples = deque(maxlen=50000)
        self._plot_samples = deque(maxlen=6000)
        self._io_results = queue.Queue()
        self._local_error = ""
        self._closing = False
        self._destroying = False
        self._close_deadline = 0.0
        self._last_state = None
        self._plot_source = None
        self._parameter_buttons = []
        self._idle_buttons = []
        self._motion_buttons = []
        self.parameter_values = {}
        self.parameter_readbacks = {}
        self.parameter_states = {}
        self._parameter_state_labels = {}
        self.flag_labels = {}
        self.metric_values = {}

        root.title("电机调试 · CAN")
        root.geometry("1400x880")
        root.minsize(1180, 780)
        root.protocol("WM_DELETE_WINDOW", self._request_close)
        configure_theme(root)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        outer = ttk.Frame(root, padding=(20, 14))
        outer.grid(sticky="nsew")
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(3, weight=1)
        heading = ttk.Frame(outer)
        heading.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        ttk.Label(heading, text="电机调试", style="Title.TLabel").pack(side="left")
        ttk.Label(heading, text="CAN 工作台", style="Muted.TLabel").pack(side="left", padx=(14, 0))
        self.phase_text = tk.StringVar(value="01 连接设备   →   02 校准   →   03 下发目标")
        ttk.Label(heading, textvariable=self.phase_text, style="Muted.TLabel").pack(side="right")
        connection = ttk.Frame(outer, padding=(0, 4))
        connection.grid(row=1, column=0, sticky="ew")
        connection.columnconfigure(0, weight=1)
        self._build_connection(connection)
        self.banner = tk.Label(
            outer, anchor="w", padx=12, pady=7, bg=COLORS["card"], fg=COLORS["amber"],
            font=("Microsoft YaHei UI", 10, "bold"),
        )
        self.banner.grid(row=2, column=0, sticky="ew", pady=(8, 14))
        workspace = ttk.Frame(outer)
        workspace.grid(row=3, column=0, sticky="nsew")
        workspace.columnconfigure(1, weight=1)
        workspace.rowconfigure(0, weight=1)
        sidebar = ttk.Frame(workspace, width=312)
        sidebar.grid(row=0, column=0, sticky="ns", padx=(0, 18))
        self._safety_host = ttk.Frame(outer)
        self._safety_host.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        self._build_controls(sidebar)
        notebook = ttk.Notebook(workspace)
        self.notebook = notebook
        notebook.grid(row=0, column=1, sticky="nsew")
        self._build_dashboard(notebook)
        self._build_parameters(notebook)
        self._build_terminal(notebook)
        self._build_frames(notebook)
        self.footer = tk.StringVar(value="上电默认停机 · 未连接 · 未收到停止确认")
        ttk.Label(outer, textvariable=self.footer, style="Muted.TLabel", font=(FONT, 9)).grid(row=5, column=0, sticky="ew", pady=(10, 0))
        self._render_snapshot({})
        self.iq_target.trace_add("write", lambda *_: self._render_snapshot(self._snapshot))
        self.root.after(50, self._poll)

    def _build_connection(self, parent):
        row = ttk.Frame(parent)
        row.grid(row=0, column=0, sticky="ew")
        row.columnconfigure(1, weight=1)
        ttk.Label(row, text="CAN 接口").grid(row=0, column=0, padx=(0, 6))
        self.interface = tk.StringVar(value=next(iter(INTERFACES)))
        combo = ttk.Combobox(row, textvariable=self.interface, values=list(INTERFACES), state="readonly", width=26)
        combo.grid(row=0, column=1, sticky="ew")
        combo.bind("<<ComboboxSelected>>", self._interface_changed)
        ttk.Label(row, text="通道").grid(row=0, column=2, padx=(10, 4))
        self.channel = tk.StringVar(value=CHANNELS[INTERFACES[self.interface.get()]])
        ttk.Entry(row, textvariable=self.channel, width=17).grid(row=0, column=3)
        ttk.Label(row, text="bit/s").grid(row=0, column=4, padx=(10, 4))
        self.bitrate = tk.StringVar(value="1000000")
        ttk.Combobox(row, textvariable=self.bitrate, values=("125000", "250000", "500000", "1000000"), width=10).grid(row=0, column=5)
        ttk.Label(row, text="连接 ID").grid(row=0, column=6, padx=(10, 4))
        self.node = tk.StringVar(value="1")
        ttk.Spinbox(row, from_=1, to=63, textvariable=self.node, width=4).grid(row=0, column=7)
        self.connect_button = RoundedButton(row, text="连接", command=self._connect, primary=True, width=90)
        self.connect_button.grid(row=0, column=8, padx=(10, 0))
        self.disconnect_button = ttk.Button(row, text="断开", command=self._disconnect)
        self.disconnect_button.grid(row=0, column=9, padx=(5, 0))
        self.recover_button = ttk.Button(row, text="恢复 PCAN", command=self._recover)
        self.recover_button.grid(row=0, column=10, padx=(5, 0))
        self._connection_widgets = [widget for widget in row.winfo_children()
                                    if isinstance(widget, (ttk.Entry, ttk.Combobox, ttk.Spinbox))]

    def _build_controls(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(2, weight=1)
        prepare = ttk.LabelFrame(parent, text="准备", padding=12)
        prepare.grid(row=0, column=0, sticky="ew")
        prepare.columnconfigure(0, weight=1)
        self.mode_text = tk.StringVar(value="未连接")
        ttk.Label(prepare, textvariable=self.mode_text, style="Heading.TLabel").grid(sticky="w")
        self.error_text = tk.StringVar()
        self._error_font = tkfont.Font(family=FONT, size=10)
        self._error_cache = (None, "")
        ttk.Label(prepare, textvariable=self.error_text, foreground=COLORS["red"], justify="left").grid(row=1, column=0, sticky="w", pady=(4, 6))
        flags = ttk.Frame(prepare)
        flags.grid(row=2, column=0, sticky="ew", pady=(2, 10))
        for index, (key, text) in enumerate((("session_ready", "CAN 会话"), ("sense_confirmed", "采样参数"),
                ("offset_ready", "偏置完成"), ("sensor_valid", "编码器"),
                ("direction_confirmed", "方向确认"), ("zero_valid", "电气零点"))):
            label = tk.Label(flags, text="○ " + text, anchor="w", bg=COLORS["bg"], fg=COLORS["muted"], font=(FONT, 9))
            label.grid(row=index//2, column=index%2, sticky="w", padx=(0, 16), pady=2)
            self.flag_labels[key] = (label, text)
        direction_row = ttk.Frame(prepare)
        direction_row.grid(row=3, column=0, sticky="ew")
        self.direction = tk.StringVar(value="+1")
        ttk.Label(direction_row, text="设置方向").pack(side="left")
        ttk.Combobox(direction_row, textvariable=self.direction, values=("+1", "-1"), state="readonly", width=4).pack(side="left", padx=7)
        direction = ttk.Button(direction_row, text="确认", command=lambda: self._submit("dir " + self.direction.get()))
        direction.pack(side="right")
        self._idle_buttons.append(direction)
        actions = ttk.Frame(prepare)
        actions.grid(row=4, column=0, sticky="ew", pady=(9, 0))
        actions.columnconfigure((0, 1), weight=1)
        calibration = ttk.Button(actions, text="重采偏置", command=lambda: self._submit("cal"))
        calibration.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.align_button = ttk.Button(actions, text="转子对齐", command=lambda: self._submit("align"))
        self.align_button.grid(row=0, column=1, sticky="ew")
        self._idle_buttons.append(calibration)
        ttk.Label(prepare, text="方向/母线修改后需重新对齐。对齐会使转子运动。", foreground=COLORS["muted"], wraplength=270, font=(FONT, 9)).grid(row=5, column=0, sticky="w", pady=(8, 0))

        control = ttk.LabelFrame(parent, text="控制", padding=14)
        control.grid(row=1, column=0, sticky="ew", pady=(14, 0))
        control.columnconfigure(0, weight=1)
        self.mode = tk.StringVar(value="iq")
        self.mode_choice = tk.StringVar(value="电流环 · Iq")
        self.mode_selector = ttk.Combobox(control, textvariable=self.mode_choice,
            values=("电流环 · Iq", "速度环 · 机械速度", "开环 · 电角速度"), state="readonly", width=25)
        self.mode_selector.grid(row=0, column=0, sticky="ew")
        self.mode_selector.bind("<<ComboboxSelected>>", self._select_mode)
        self.iq_target = tk.StringVar(value="0.050")
        self.speed_target = tk.StringVar(value="1.000")
        self.open_target = tk.StringVar(value="20.000")
        self.open_voltage = tk.StringVar(value="0.800")
        self.target_unit = tk.StringVar(value=f"目标 Iq / A  ·  输入 ±{HOST_CURRENT_LIMIT_A:g} A")
        ttk.Label(control, textvariable=self.target_unit, foreground=COLORS["muted"]).grid(row=1, column=0, sticky="w", pady=(12, 4))
        self.target_entry = ttk.Spinbox(control, from_=-HOST_CURRENT_LIMIT_A, to=HOST_CURRENT_LIMIT_A, increment=.01, textvariable=self.iq_target, font=("Consolas", 20), width=14)
        self.target_entry.grid(row=2, column=0, sticky="ew")
        self.current_limit_text = tk.StringVar(value="可在参数页读取板端目标上限。")
        self.current_limit_label = ttk.Label(control, textvariable=self.current_limit_text,
            foreground=COLORS["muted"], wraplength=270, font=(FONT, 9))
        self.current_limit_label.grid(row=3, column=0, sticky="w", pady=(8, 0))
        self.open_voltage_row = ttk.Frame(control)
        ttk.Label(self.open_voltage_row, text="开环电压 / V").pack(side="left")
        ttk.Spinbox(self.open_voltage_row, from_=0, to=2, increment=.1, textvariable=self.open_voltage, width=7).pack(side="right")
        self.start_button = RoundedButton(control, text="启动控制", command=self._start_target, primary=True, width=220)
        self.start_button.grid(row=4, column=0, sticky="ew", pady=(12, 0))
        self._motion_buttons.append(self.start_button)
        self.start_hint = tk.StringVar(value="先连接设备并完成启动条件。")
        ttk.Label(control, textvariable=self.start_hint, foreground=COLORS["muted"], wraplength=270, font=(FONT, 9)).grid(row=5, column=0, sticky="w", pady=(8, 0))

        # Keep stop controls outside the shrinking workspace: a long diagnostic
        # or a short window must never push them below the visible client area.
        safety = ttk.Frame(self._safety_host)
        safety.grid(row=0, column=0, sticky="w")
        safety.columnconfigure((0, 1), weight=1)
        self.stop_button = RoundedButton(safety, text="停止输出", command=self._stop, danger=True)
        self.stop_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.estop_button = RoundedButton(safety, text="急停锁存", command=self._estop)
        self.estop_button.grid(row=0, column=1, sticky="ew")
        self.clear_button = ttk.Button(safety, text="清除锁存故障（不自动重启）", command=lambda: self._submit("clear"))
        self.clear_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    def _select_mode(self, event=None):
        selection = {"电流环 · Iq": ("iq", self.iq_target, -HOST_CURRENT_LIMIT_A, HOST_CURRENT_LIMIT_A, .01,
                                  f"目标 Iq / A  ·  输入 ±{HOST_CURRENT_LIMIT_A:g} A"),
                     "速度环 · 机械速度": ("speed", self.speed_target, -30, 30, .1, "机械速度 / rad/s"),
                     "开环 · 电角速度": ("open", self.open_target, -100, 100, 1, "电角速度 / rad/s")}
        mode, variable, low, high, step, unit = selection[self.mode_choice.get()]
        self.mode.set(mode)
        self.target_entry.configure(textvariable=variable, from_=low, to=high, increment=step)
        self.target_unit.set(unit)
        if mode == "open":
            self.open_voltage_row.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        else:
            self.open_voltage_row.grid_remove()
        if mode == "iq":
            self.current_limit_label.grid(row=3, column=0, sticky="w", pady=(8, 0))
        else:
            self.current_limit_label.grid_remove()
        self._render_snapshot(self._snapshot)

    def _build_dashboard(self, notebook):
        page = ttk.Frame(notebook)
        page.columnconfigure(0, weight=1)
        page.rowconfigure(1, weight=1)
        metrics = ttk.Frame(page)
        metrics.grid(row=0, column=0, sticky="ew", pady=(4, 14))
        for index, (key, label, color) in enumerate((("iq_a", "实测 Iq / A", COLORS["accent"]),
                ("iq_reference_a", "参考 Iq / A", COLORS["amber"]), ("id_a", "实测 Id / A", COLORS["purple"]),
                ("rpm", "机械转速 / rpm", COLORS["blue"]))):
            metrics.columnconfigure(index, weight=1, uniform="top")
            self.metric_values[key] = tk.StringVar(value="—")
            MetricCard(metrics, label, self.metric_values[key], color).grid(row=0, column=index, sticky="nsew", padx=(0, 8 if index < 3 else 0))
        middle = ttk.Frame(page)
        middle.grid(row=1, column=0, sticky="nsew")
        middle.columnconfigure(1, weight=1)
        middle.rowconfigure(0, weight=1)
        encoder = ttk.Frame(middle, style="Card.TFrame", padding=(16, 14))
        encoder.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        encoder.rowconfigure(1, weight=1)
        ttk.Label(encoder, text="编码器位置", style="Card.TLabel", font=(FONT, 11, "bold")).grid(row=0, column=0, sticky="w")
        self.encoder_dial = EncoderDial(encoder)
        self.encoder_dial.grid(row=1, column=0, sticky="nsew")
        self.angle_radians = tk.StringVar(value="— rad")
        self.angle_quality = tk.StringVar(value="未连接")
        self.angle_detail = tk.StringVar(value="机械角 · page 2 · 约 33 Hz")
        ttk.Label(encoder, textvariable=self.angle_radians, style="Card.TLabel", font=("Consolas", 13)).grid(row=2, column=0)
        self.encoder_quality_label = ttk.Label(encoder, textvariable=self.angle_quality, style="Card.TLabel", foreground=COLORS["muted"])
        self.encoder_quality_label.grid(row=3, column=0, pady=(8, 0))
        ttk.Label(encoder, textvariable=self.angle_detail, style="Card.TLabel", foreground=COLORS["muted"], font=(FONT, 9)).grid(row=4, column=0, pady=(6, 0))
        plot = ttk.Frame(middle)
        plot.grid(row=0, column=1, sticky="nsew")
        self._build_plot(plot)
        details = ttk.Frame(page)
        details.grid(row=2, column=0, sticky="ew", pady=(14, 0))
        for index, (key, label) in enumerate((("voltage", "Vd / Vq  ·  V"), ("raw", "ADC 原始值  ·  U / W"),
                ("bus", "配置母线电压  ·  V"), ("isr", "控制 ISR 峰值  ·  μs"))):
            self.metric_values[key] = tk.StringVar(value="—")
            details.columnconfigure(index, weight=1, uniform="bottom")
            MetricCard(details, label, self.metric_values[key], small=True).grid(row=0, column=index, sticky="nsew", padx=(0, 8 if index < 3 else 0))
        notebook.add(page, text="监测总览")

    def _build_plot(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(2, weight=1)
        toolbar = ttk.Frame(parent)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Label(toolbar, text="实时趋势", style="Heading.TLabel").pack(side="left")
        self.plot_kind = tk.StringVar(value="电流 / A")
        kind = ttk.Combobox(toolbar, textvariable=self.plot_kind, values=("电流 / A", "角度 / °", "转速 / rpm"), state="readonly", width=12)
        kind.pack(side="left", padx=(12, 0))
        kind.bind("<<ComboboxSelected>>", lambda event: self._draw_plot())
        ttk.Button(toolbar, text="导出 CSV", command=self._export_csv).pack(side="right")
        ttk.Button(toolbar, text="清空", command=self._clear_plot).pack(side="right", padx=5)
        legend = ttk.Frame(parent)
        legend.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        self.visible_curves = {}
        for key, name, color in (("iq_reference_a", "IqRef", COLORS["amber"]), ("iq_a", "Iq", COLORS["accent"]), ("id_a", "Id", COLORS["purple"])):
            visible = tk.BooleanVar(value=True)
            self.visible_curves[key] = visible, name, color
            ttk.Checkbutton(legend, text=name, variable=visible, command=self._draw_plot).pack(side="left", padx=(0, 8))
        self.plot_span = tk.StringVar(value="10")
        span = ttk.Combobox(legend, textvariable=self.plot_span, values=("5", "10", "30"), state="readonly", width=4)
        span.pack(side="right")
        span.bind("<<ComboboxSelected>>", lambda event: self._draw_plot())
        ttk.Label(legend, text="窗口 / s", foreground=COLORS["muted"]).pack(side="right", padx=6)
        self.canvas = tk.Canvas(parent, height=320, background=COLORS["bg"], highlightthickness=0)
        self.canvas.grid(row=2, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda event: self._draw_plot())
        ttk.Label(parent, text="CAN 监视数据 · 各遥测页非同步采样", foreground=COLORS["muted"], font=(FONT, 9)).grid(row=3, column=0, sticky="w", pady=(6, 0))

    def _build_terminal(self, notebook):
        page = ttk.Frame(notebook, padding=8)
        page.columnconfigure(0, weight=1)
        page.rowconfigure(0, weight=1)
        self.terminal = ScrolledText(page, height=8, state="disabled", wrap="word", font=("Consolas", 10), background=COLORS["input"], foreground=COLORS["text"], insertbackground=COLORS["text"], relief="flat", borderwidth=0, padx=12, pady=10)
        self.terminal.grid(row=0, column=0, columnspan=2, sticky="nsew")
        self.command = tk.StringVar()
        entry = ttk.Entry(page, textvariable=self.command)
        entry.grid(row=1, column=0, sticky="ew", pady=(7, 0))
        entry.bind("<Return>", lambda event: self._terminal_submit())
        ttk.Button(page, text="发送命令", command=self._terminal_submit).grid(row=1, column=1, padx=(7, 0), pady=(7, 0))
        ttk.Label(page, text="help / status / stop / estop / clear / cal / dir / align / iq / speed / open / bus / get / set / node", foreground=COLORS["muted"]).grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 0))
        notebook.add(page, text="  命令终端  ")

    def _build_frames(self, notebook):
        page = ttk.Frame(notebook, padding=8)
        page.columnconfigure(0, weight=1)
        page.rowconfigure(0, weight=1)
        self.frames = ScrolledText(page, height=8, state="disabled", wrap="none", font=("Consolas", 10), background=COLORS["input"], foreground=COLORS["text"], relief="flat", borderwidth=0, padx=12, pady=10)
        self.frames.grid(sticky="nsew")
        ttk.Label(page, text="最近 600 行 · TX 发送 / RX 接收 · 标准 CAN ID 与十六进制数据", foreground=COLORS["muted"]).grid(row=1, column=0, sticky="w", pady=(4, 0))
        notebook.add(page, text="  CAN 帧  ")

    def _build_parameters(self, notebook):
        container = ttk.Frame(notebook)
        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)
        canvas = tk.Canvas(container, highlightthickness=0, background=COLORS["bg"])
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(container, orient="horizontal", command=canvas.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        canvas.configure(xscrollcommand=horizontal.set)
        page = ttk.Frame(canvas, padding=8)
        window = canvas.create_window((0, 0), window=page, anchor="nw")
        page.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        def resize_parameter_page(event):
            canvas.itemconfigure(window, width=max(event.width, 900))
            if event.width < 900:
                horizontal.grid(row=1, column=0, sticky="ew")
            else:
                horizontal.grid_remove()
                canvas.xview_moveto(0)
        canvas.bind("<Configure>", resize_parameter_page)
        page.columnconfigure(2, weight=1)
        toolbar = ttk.Frame(page)
        toolbar.grid(row=0, column=0, columnspan=8, sticky="ew")
        self.read_parameters_button = ttk.Button(toolbar, text="读取板上参数", command=lambda: self._invoke("read_parameters"))
        self.read_parameters_button.pack(side="left")
        self.adopt_parameters_button = ttk.Button(toolbar, text="读回填入编辑器", command=self._adopt_readbacks)
        self.adopt_parameters_button.pack(side="left", padx=6)
        self.write_parameters_button = ttk.Button(toolbar, text="写入修改项", style="Accent.TButton", command=self._apply_changed_parameters)
        self.write_parameters_button.pack(side="left", padx=(0, 14))
        plans = ttk.Frame(page)
        plans.grid(row=1, column=0, columnspan=8, sticky="ew", pady=(9, 4))
        ttk.Button(plans, text="保存编辑值方案…", command=self._save_profile).pack(side="left")
        ttk.Button(plans, text="导入到编辑器…", command=self._load_profile).pack(side="left", padx=6)
        ttk.Label(plans, text="电机标签").pack(side="left", padx=(8, 4))
        self.motor_label = tk.StringVar(value="4015 / 14 V")
        ttk.Entry(plans, textvariable=self.motor_label, width=20).pack(side="left", fill="x", expand=True)
        self.parameter_summary = tk.StringVar(value="编辑值为本机默认值；尚未读取板上参数。")
        adopted = ttk.Frame(page)
        adopted.grid(row=2, column=0, columnspan=8, sticky="ew", pady=(7, 2))
        ttk.Label(adopted, textvariable=self.parameter_summary, foreground=COLORS["muted"], wraplength=650).pack(side="left", fill="x", expand=True)
        ttk.Label(page, text="方案只存电脑，导入只填编辑器。STOP 后逐项写入并读回，已确认项不自动回滚。", foreground=COLORS["muted"], wraplength=650).grid(row=3, column=0, columnspan=8, sticky="w", pady=(0, 5))
        for column, title in enumerate(("ID", "分组", "参数", "单位", "编辑值", "板上读回", "状态", "操作")):
            ttk.Label(page, text=title, foreground=COLORS["muted"]).grid(row=4, column=column, sticky="w", padx=(0, 12), pady=(4, 6))
        for index, spec in enumerate(PARAMETER_SPECS):
            row, identifier = 5 + 2 * index, spec.id
            ttk.Label(page, text=str(identifier)).grid(row=row, column=0, sticky="w")
            group_label = {"current": "电流环", "speed": "速度环", "system": "系统"}.get(spec.group, spec.group)
            ttk.Label(page, text=group_label, foreground=COLORS["muted"]).grid(row=row, column=1, sticky="w", padx=(0, 12))
            ttk.Label(page, text=spec.label).grid(row=row, column=2, sticky="w", padx=(0, 12))
            ttk.Label(page, text=spec.unit or "—").grid(row=row, column=3, sticky="w", padx=(0, 12))
            value = tk.StringVar(value=f"{spec.default:g}")
            readback = tk.StringVar(value="—")
            state = tk.StringVar(value="未读回")
            self.parameter_values[identifier] = value
            self.parameter_readbacks[identifier] = readback
            self.parameter_states[identifier] = state
            ttk.Spinbox(page, from_=spec.minimum, to=spec.maximum, increment=spec.step, textvariable=value, width=11).grid(row=row, column=4, sticky="w", padx=(0, 12), pady=1)
            ttk.Label(page, textvariable=readback, width=11, font=("Consolas", 10)).grid(row=row, column=5, sticky="w", padx=(0, 12))
            state_label = ttk.Label(page, textvariable=state, width=8, foreground=COLORS["muted"])
            state_label.grid(row=row, column=6, sticky="w", padx=(0, 12))
            self._parameter_state_labels[identifier] = state_label
            button = ttk.Button(page, text="应用此项", command=lambda ident=identifier: self._apply_parameter(ident))
            button.grid(row=row, column=7, sticky="e", pady=1)
            self._parameter_buttons.append(button)
            suffix = " 修改后需重新对齐。" if spec.invalidates_zero else ""
            ttk.Label(page, text=f"范围 {spec.minimum:g} … {spec.maximum:g}；{spec.description}{suffix}", foreground=COLORS["muted"], wraplength=650).grid(row=row + 1, column=1, columnspan=7, sticky="w", pady=(0, 9))
            value.trace_add("write", lambda *args: self._refresh_parameter_states())
        address = ttk.LabelFrame(page, text='板上节点 ID · 掉电保存', padding=10)
        address.grid(row=5 + 2*len(PARAMETER_SPECS), column=0, columnspan=8, sticky='ew', pady=(12, 6))
        self.node_info_text = tk.StringVar(value='尚未读取板上 ID；顶部“连接 ID”只选择通信地址。')
        ttk.Label(address, textvariable=self.node_info_text, foreground=COLORS['muted']).grid(row=0, column=0, columnspan=5, sticky='w')
        ttk.Label(address, text='新 ID').grid(row=1, column=0, sticky='w', pady=(8, 0))
        self.new_node = tk.StringVar(value='3')
        self.new_node_editor = ttk.Spinbox(address, from_=1, to=63, textvariable=self.new_node, width=6)
        self.new_node_editor.grid(row=1, column=1, padx=(6, 12), pady=(8, 0))
        self.read_node_button = ttk.Button(address, text='读取板上 ID', command=lambda: self._invoke('read_node'))
        self.read_node_button.grid(row=1, column=2, padx=(0, 8), pady=(8, 0))
        self.single_board = tk.BooleanVar(value=False)
        self.single_board_check = ttk.Checkbutton(address, text='当前只连接这一块驱动板', variable=self.single_board,
                                                  command=lambda: self._render_snapshot(self._snapshot))
        self.single_board_check.grid(row=1, column=3, padx=(0, 12), pady=(8, 0))
        self.save_node_button = ttk.Button(address, text='保存 ID 并重连', command=self._save_node)
        self.save_node_button.grid(row=1, column=4, pady=(8, 0))
        ttk.Label(address, text='保存会重启驱动板并重采偏置；若已有匹配的电角度校准，会自动恢复。其它调参值仍只保存在 RAM。',
                  foreground=COLORS['muted'], wraplength=750).grid(row=2, column=0, columnspan=5, sticky='w', pady=(8, 0))
        calibration = ttk.LabelFrame(page, text='电角度零点 · Flash 掉电保存', padding=10)
        calibration.grid(row=6 + 2*len(PARAMETER_SPECS), column=0, columnspan=8,
                         sticky='ew', pady=(6, 12))
        self.calibration_info_text = tk.StringVar(
            value='尚未读取。第一次空载对齐成功后再保存；装车带载时不要重新对齐。')
        ttk.Label(calibration, textvariable=self.calibration_info_text,
                  foreground=COLORS['muted'], wraplength=760).grid(
                      row=0, column=0, columnspan=3, sticky='w')
        self.read_calibration_button = ttk.Button(
            calibration, text='读取保存状态', command=lambda: self._invoke('read_calibration'))
        self.read_calibration_button.grid(row=1, column=0, sticky='w', pady=(8, 0))
        self.save_calibration_button = ttk.Button(
            calibration, text='保存当前有效零点并重启', style='Accent.TButton',
            command=lambda: self._invoke('save_calibration'))
        self.save_calibration_button.grid(row=1, column=1, sticky='w', padx=8, pady=(8, 0))
        self.clear_calibration_button = ttk.Button(
            calibration, text='拆板/换电机：清除保存值', command=self._confirm_clear_calibration)
        self.clear_calibration_button.grid(row=1, column=2, sticky='w', pady=(8, 0))
        ttk.Label(calibration,
                  text='保存只接受：驱动关闭、STOP 已确认、编码器有效、方向已确认且零点有效。写入后上电不再吸合，只恢复零点。',
                  foreground=COLORS['muted'], wraplength=760).grid(
                      row=2, column=0, columnspan=3, sticky='w', pady=(8, 0))
        notebook.add(container, text="  参数调试  ")

    def _invoke(self, method, *args, **kwargs):
        if self._destroying:
            return False
        try:
            getattr(self.controller, method)(*args, **kwargs)
            self._local_error = ""
            return True
        except Exception as error:
            self._show_error(str(error))
            return False

    def _show_error(self, text):
        self._local_error = str(text)
        self.error_text.set(self._compact_error(self._local_error))
        self._log(self._local_error)

    def _compact_error(self, text):
        """Bound sidebar diagnostics to three lines; full errors stay in the log."""
        text = str(text or "")
        if self._error_cache[0] == text:
            return self._error_cache[1]
        lines = [""]
        for character in text:
            if character == '\n' or self._error_font.measure(lines[-1] + character) > 270:
                if len(lines) == 3:
                    suffix = '…（详见终端）'
                    while lines[-1] and self._error_font.measure(lines[-1] + suffix) > 270:
                        lines[-1] = lines[-1][:-1]
                    lines[-1] += suffix
                    break
                lines.append("")
            if character != '\n':
                lines[-1] += character
        preview = '\n'.join(lines)
        self._error_cache = (text, preview)
        return preview

    def _interface_changed(self, event=None):
        interface = INTERFACES[self.interface.get()]
        if self.channel.get() in set(CHANNELS.values()) or not self.channel.get().strip():
            self.channel.set(CHANNELS[interface])
        self._render_snapshot(self._snapshot)

    def _connect(self):
        self._request_connection("connect")

    def _recover(self):
        self._request_connection("recover")

    def _save_node(self):
        try:
            new_node = int(self.new_node.get())
            if not 1 <= new_node <= 63:
                raise ValueError('节点 ID 范围为 1 … 63。')
            if not self.single_board.get():
                raise ValueError('请只连接一块驱动板并勾选单板确认。')
            self._invoke('change_node', new_node, single_board=True)
        except ValueError as error:
            self._show_error(str(error))

    def _confirm_clear_calibration(self):
        confirmed = messagebox.askyesno(
            '清除电角度校准',
            '仅在拆下电机板、换电机、改相线或改变编码器安装后清除。\n\n'
            '清除会写入 Flash 并重启驱动板；不会启动电机。下次真正掉电上电时，'
            '若转子可安全移动，固件会重新执行一次自动吸合对齐。\n\n确认清除吗？',
            parent=self.root,
        )
        if confirmed:
            self._invoke('clear_calibration')

    def _request_connection(self, action):
        try:
            bitrate = int(self.bitrate.get())
            node = int(self.node.get())
            if bitrate <= 0 or not 1 <= node <= 63:
                raise ValueError("波特率必须为正整数，节点范围为 1 … 63。")
            config = {"interface": INTERFACES[self.interface.get()], "channel": self.channel.get().strip(), "bitrate": bitrate, "node": node}
            if not config["channel"]:
                raise ValueError("请填写 CAN 通道。")
            self._handle_events(self.controller.events() or [])
            self._plot_samples.clear()
            self._plot_source = config["interface"]
            self._invoke(action, config)
            self._log("已请求恢复 PCAN：重启适配器并重新 STOP 握手，保持停机。" if action == "recover" else
                      "已提交连接请求；连接成功不代表电机已启动。")
        except (ValueError, KeyError) as error:
            self._show_error(str(error))

    def _disconnect(self):
        self._invoke("stop")
        self._invoke("disconnect")
        self._log("已请求停止并断开，停止结果以执行确认显示为准。")

    def _stop(self):
        self._invoke("stop")
        self._log("→ STOP：已提交，等待执行确认。")

    def _estop(self):
        self._invoke("estop")
        self._log("→ ESTOP：已提交急停锁存请求。")

    def _submit(self, text):
        if self._closing:
            return
        if not self._snapshot.get("connected"):
            self._show_error("请先连接模拟器或 CAN 接口。")
            return
        if not self._iq_target_allowed(text):
            return
        if self._invoke("submit", text):
            self._log(f"→ {text}")

    def _iq_target_allowed(self, text):
        """Block a direct iq target above the real read-back limit with a popup.

        Only parameter 3 that was actually read back counts; an unknown limit is
        left to the board's execution ACK. A target above the read-back limit is
        refused here so the operator sees the real +/-N A limit immediately.
        """
        fields = text.split()
        if len(fields) != 2 or fields[0].lower() != "iq":
            return True
        value = _number(fields[1])
        if value is None:
            return True
        try:
            validate_current_target(value, self._snapshot.get("parameters") or {},
                                    require_readback=False)
        except ValueError as error:
            message = str(error)
            self._show_error(message)
            if "电流限幅" in message:
                messagebox.showwarning("目标电流超出板端限幅", message, parent=self.root)
            return False
        return True

    def _validated(self, variable, low, high, label):
        value = _number(variable.get())
        if value is None or not low <= value <= high:
            raise ValueError(f"{label}必须在 {low:g} … {high:g} 之间。")
        return value

    def _start_target(self):
        try:
            mode = self.mode.get()
            if mode == "iq":
                value = self._validated(self.iq_target, -HOST_CURRENT_LIMIT_A, HOST_CURRENT_LIMIT_A, "Iq (A)")
                text = f"iq {value:.6f}"
            elif mode == "speed":
                value = self._validated(self.speed_target, -30.0, 30.0, "机械速度 (rad/s)")
                text = f"speed {value:.6f}"
            else:
                velocity = self._validated(self.open_target, -100.0, 100.0, "电角速度 (rad/s)")
                voltage = self._validated(self.open_voltage, 0.0, 2.0, "开环电压 (V)")
                text = f"open {velocity:.6f} {voltage:.6f}"
            self._submit(text)
        except ValueError as error:
            self._show_error(str(error))

    def _terminal_submit(self):
        text = self.command.get().strip()
        if not text:
            return
        self.command.set("")
        if text.lower() == "stop":
            self._stop()
        elif text.lower() == "estop":
            self._estop()
        else:
            self._submit(text)

    def _apply_parameter(self, identifier):
        if not self._parameter_idle():
            self._show_error("参数只能在已连接、驱动停止且无其他参数读写时应用。")
            return
        spec = _SPECS_BY_ID[identifier]
        try:
            value = self._validated(self.parameter_values[identifier], spec.minimum, spec.maximum, spec.label)
            if self._invoke("set_parameter", identifier, value):
                self._log(f"→ set {identifier} {value:.6f}：等待执行与读回。")
        except ValueError as error:
            self._show_error(str(error))

    def _parameter_idle(self):
        status = self._snapshot.get("status") or {}
        faults = (self._snapshot.get("diagnostic") or {}).get("faults", 0)
        return bool(not faults and self._snapshot.get("connected") and not self._snapshot.get("running") and
                    not status.get("driver_enabled") and not self._snapshot.get("tuning_busy") and not self._closing and
                    not any(self._snapshot.get(k) for k in ('connecting', 'recovering', 'reconnect_pending', 'node_changing')))

    def _readback_values(self):
        if not self._snapshot.get("connected"):
            return {}
        result = {}
        for key, value in (self._snapshot.get("parameters") or {}).items():
            try:
                identifier = int(key)
            except (TypeError, ValueError):
                continue
            number = _number(value)
            if identifier in PARAMETERS and number is not None:
                result[identifier] = number
        return result

    def _editor_parameters(self):
        result = {}
        for spec in PARAMETER_SPECS:
            result[spec.name] = self._validated(self.parameter_values[spec.id], spec.minimum, spec.maximum, spec.label)
        return validate_parameters(result)

    def _changed_parameters(self):
        edited = self._editor_parameters()
        actual = self._readback_values()
        if any(spec.id not in actual for spec in PARAMETER_SPECS):
            raise ValueError("请先读取全部板上参数，再按实际读回值计算修改项。")
        return {spec.id: edited[spec.name] for spec in PARAMETER_SPECS
                if not float32_equal(edited[spec.name], actual[spec.id])}

    def _refresh_parameter_states(self):
        actual = self._readback_values()
        changed = invalid = 0
        for identifier, variable in self.parameter_values.items():
            spec = _SPECS_BY_ID[identifier]
            value = _number(variable.get())
            if value is None or not spec.minimum <= value <= spec.maximum:
                text, color = "输入无效", COLORS["red"]
                invalid += 1
            elif identifier not in actual:
                text, color = "未读回", COLORS["muted"]
            elif float32_equal(value, actual[identifier]):
                text, color = "已同步", COLORS["green"]
            else:
                text, color = "已修改", COLORS["amber"]
                changed += 1
            self.parameter_states[identifier].set(text)
            self._parameter_state_labels[identifier].configure(foreground=color)
        if self._snapshot.get("tuning_busy"):
            text = "正在读写参数并等待实际读回；已确认项会逐项更新。"
        else:
            text = f"板上读回 {len(actual)}/{len(PARAMETER_SPECS)} 项 · 已修改 {changed} 项"
            if invalid:
                text += f" · 输入无效 {invalid} 项"
            if not actual:
                text += " · 当前数值是编辑器内容，尚未确认板上实际值。"
        self.parameter_summary.set(text)
        can_write = self._parameter_idle() and changed > 0 and not invalid and len(actual) == len(PARAMETER_SPECS)
        self.write_parameters_button.configure(state="normal" if can_write else "disabled")
        can_adopt = self._parameter_idle() and len(actual) == len(PARAMETER_SPECS)
        self.adopt_parameters_button.configure(state="normal" if can_adopt else "disabled")

    def _adopt_readbacks(self):
        if not self._parameter_idle():
            self._show_error("请先 STOP，并等待当前参数操作完成。")
            return
        actual = self._readback_values()
        if any(spec.id not in actual for spec in PARAMETER_SPECS):
            self._show_error("请先读取全部板上参数，再填入编辑器。")
            return
        try:
            named = {}
            for spec in PARAMETER_SPECS:
                value = actual[spec.id]
                # A valid float32 minimum such as 0.01 can decode a few ulps
                # below the decimal schema boundary. Adopt its canonical bound.
                if float32_equal(value, spec.minimum):
                    value = spec.minimum
                elif float32_equal(value, spec.maximum):
                    value = spec.maximum
                named[spec.name] = value
            validate_parameters(named)  # Validate all values before touching any editor.
            for spec in PARAMETER_SPECS:
                value = named[spec.name]
                for digits in range(1, 10):
                    text = f"{value:.{digits}g}"
                    parsed = float(text)
                    if spec.minimum <= parsed <= spec.maximum and float32_equal(value, parsed):
                        break
                self.parameter_values[spec.id].set(text)
            self._refresh_parameter_states()
            self._log("已将完整板上读回填入编辑器；未发送 CAN、未启动电机。")
        except ValueError as error:
            self._show_error(str(error))

    def _apply_changed_parameters(self):
        if not self._parameter_idle():
            self._show_error("请先 STOP，并等待当前参数操作完成。")
            return
        try:
            changed = self._changed_parameters()
            if not changed:
                self._log("编辑值与板上读回值在 float32 精度下相同，无需写入。")
                return
            if self._invoke("set_parameters", changed):
                self._log(f"→ 写入 {len(changed)} 个修改项，逐项等待 ACK 和读回；不会启动电机。")
        except ValueError as error:
            self._show_error(str(error))

    def _save_profile(self):
        try:
            parameters = self._editor_parameters()
        except ValueError as error:
            self._show_error(str(error))
            return
        path = filedialog.asksaveasfilename(parent=self.root, title="保存编辑值方案（仅电脑，不写入电机）", defaultextension=".json", filetypes=(("JSON 参数方案", "*.json"),), initialfile="motor_parameters.json")
        if not path:
            return
        motor = self.motor_label.get().strip() or None
        metadata = {"interface": self._snapshot.get("interface") or "offline",
                    "note": "保存编辑器值；不声明为板上实际参数。"}

        def write_profile():
            try:
                save_profile(path, parameters, motor=motor, metadata=metadata)
                self._io_results.put((True, f"编辑值方案已保存至电脑：{path}；未发送 CAN。"))
            except (ProfileError, OSError) as error:
                self._io_results.put((False, str(error)))

        threading.Thread(target=write_profile, name="foc-profile-save", daemon=True).start()

    def _load_profile(self):
        path = filedialog.askopenfilename(parent=self.root, title="导入方案到编辑器（不会自动发 CAN）", filetypes=(("JSON 参数方案", "*.json"),))
        if not path:
            return

        def read_profile():
            try:
                profile = load_profile(path)
                self._io_results.put(("profile_loaded", (profile, path)))
            except (ProfileError, OSError) as error:
                self._io_results.put((False, str(error)))

        threading.Thread(target=read_profile, name="foc-profile-load", daemon=True).start()

    def _import_profile(self, profile, path=""):
        # load_profile validated every key/value before this single Tk update.
        # A partial plan touches only its listed editors and sends no CAN API.
        for identifier, value in profile.to_mapping().items():
            self.parameter_values[identifier].set(f"{value:.9g}")
        if profile.motor is not None:
            self.motor_label.set(profile.motor)
        self._refresh_parameter_states()
        self._log(f"已导入 {len(profile.parameters)} 项到编辑器：{path}；未写入板上、未启动电机。")

    def _render_snapshot(self, snapshot):
        self._snapshot = snapshot
        connected, running = bool(snapshot.get("connected")), bool(snapshot.get("running"))
        recovering = bool(snapshot.get("recovering"))
        pending_reconnect = bool(snapshot.get("reconnect_pending"))
        node_changing = bool(snapshot.get('node_changing'))
        busy = recovering or bool(snapshot.get("connecting")) or pending_reconnect or node_changing
        now = time.monotonic()
        received = snapshot.get("received") or {}
        status_fresh = connected and 0 <= now - received.get("status", -1e9) <= DISPLAY_TIMEOUT_S
        diagnostic_fresh = connected and 0 <= now - received.get("diagnostic", -1e9) <= DISPLAY_TIMEOUT_S
        feedback_fresh = status_fresh and diagnostic_fresh
        link_error = can_feedback_error(snapshot, now)
        status = (snapshot.get("status") or {}) if status_fresh else {}
        telemetry = snapshot.get("telemetry") or {}
        diagnostic = snapshot.get("diagnostic") or {}
        session_ready = bool(snapshot.get("session_ready")) and feedback_fresh and not any(
            diagnostic.get(key) for key in ("require_stop", "not_initialized", "bus_off", "error_passive"))
        current_parameters = (snapshot.get("parameters") or {}) if session_ready else {}
        current_limit = current_limit_readback(current_parameters)
        limits = (snapshot.get('limits_info') or {}) if session_ready else {}
        phase_trip = _number(limits.get('phase_trip_a'))
        if current_limit is not None:
            self.current_limit_text.set(f"板端目标上限：±{current_limit:g} A\n" +
                (f"相电流保护：{phase_trip:g} A（触发即停机）" if phase_trip is not None and phase_trip > 0 else
                 "相电流保护：未读回"))
        else:
            self.current_limit_text.set("板端目标上限未确认；可读取板上参数。")
        active_interface = (snapshot.get("interface") if connected else None) or INTERFACES[self.interface.get()]
        if connected and active_interface != self._plot_source:
            self._plot_samples.clear()
            self._plot_source = active_interface
        simulator = active_interface == "simulator"
        if simulator:
            banner = "模拟器 · 演示数据，角度不来自板子"
            foreground = COLORS["amber"]
        else:
            connection = "反馈正常" if feedback_fresh else "反馈中断" if connected else "未连接"
            banner = f"{active_interface.upper()} · {connection}"
            foreground = COLORS["red"] if link_error or snapshot.get("error") else COLORS["accent"]
        if busy:
            banner = ('修改节点 ID' if node_changing else "自动重连" if pending_reconnect else "恢复 PCAN" if recovering else "正在连接") + " · " + snapshot.get("connection_stage", "")
            foreground = COLORS["amber"]
        if self._closing:
            banner = "正在关闭 / 等待 STOP 执行确认，然后断开"
        self.banner.configure(text=banner, bg=COLORS["card"], fg=foreground)
        mode_names = {"STOP": "停止", "CURRENT": "电流环", "SPEED": "速度环", "OPEN": "开环", "ALIGN": "对齐", "FAULT": "故障"}
        mode = snapshot.get("mode", "DISCONNECTED")
        enabled = bool(status.get("driver_enabled"))
        self.mode_text.set((mode_names.get(mode, mode) + " · " + ("驱动已使能" if enabled else "驱动关闭")) if status_fresh else "等待反馈" if connected else "未连接")
        faults = diagnostic.get("faults", 0) if connected else 0
        error = snapshot.get("error") or self._local_error or link_error
        if not error and feedback_fresh and not status.get("sensor_valid"):
            error = "编码器反馈无效，角度不可用；CAN 反馈正常，请检查磁铁位置和 I²C。"
        if faults:
            error = f"故障 0x{faults:08X}（{describe_faults(faults)}）  {error}"
        self.error_text.set(self._compact_error(error))

        def trace_value(key, page):
            fresh = status_fresh and 0 <= now - received.get(f"trace_{page}", -1e9) <= DISPLAY_TIMEOUT_S
            return telemetry.get(key) if fresh else None

        self.metric_values["iq_a"].set(_display(status.get("iq_a"), 3))
        self.metric_values["id_a"].set(_display(trace_value("id_a", 0), 3))
        self.metric_values["iq_reference_a"].set(_display(trace_value("iq_reference_a", 0), 3))
        self.metric_values["voltage"].set(f"{_display(trace_value('vd_v', 0), 2)} / {_display(trace_value('vq_v', 1), 2)}")
        self.metric_values["raw"].set(f"{_display(trace_value('raw_u', 1), 0)} / {_display(trace_value('raw_w', 1), 0)}")
        self.metric_values["bus"].set(_display(trace_value("bus_voltage_v", 2), 2))
        self.metric_values["isr"].set(_display(trace_value("max_isr_us", 2), 0))
        encoder = encoder_readout(snapshot, now)
        if not error and encoder.quality == "angle_stale":
            source = f"0x{0x700 + snapshot.get('node', 1):03X} 快速角度帧" if 'fast' in received else f"角度反馈页 0x{0x580 + status.get('node', snapshot.get('node', 1)):03X} / page 2"
            self.error_text.set(self._compact_error(f"{source}已过期；状态帧仍在接收。"))
        self.metric_values["rpm"].set(_display(encoder.rpm, 1))
        self.encoder_dial.update_readout(encoder)
        self.angle_radians.set(_display(encoder.angle_rad, 3, " rad"))
        qualities = {"valid": "编码器数据有效", "disconnected": "未连接设备", "waiting_status": "等待状态反馈",
                     "status_stale": "状态反馈已过期", "sensor_invalid": "编码器未就绪 / 无效",
                     "waiting_angle": "等待角度页反馈", "angle_stale": "角度反馈已过期"}
        self.angle_quality.set("模拟角度 · 不读取板子" if simulator and encoder.valid else qualities[encoder.quality])
        self.encoder_quality_label.configure(foreground=COLORS["accent"] if encoder.valid else COLORS["amber"])
        node = status.get("node", snapshot.get('node', 1))
        self.angle_detail.set(("SIM / 模拟数据 · " if simulator else f"0x{0x700+node:03X} / 快速反馈 · " if 'fast' in received else f"0x{0x580+node:03X} / page 2 · ") +
                              (f"{encoder.age_ms:.0f} ms" if encoder.age_ms is not None else "约 200 Hz" if 'fast' in received else "约 33 Hz"))
        for key, (label, text) in self.flag_labels.items():
            value = session_ready if key == "session_ready" else bool(status.get(key))
            label.configure(text=f"{'●' if value else '○'} {text}", fg=COLORS["green"] if value else COLORS["muted"])
        actual_parameters = self._readback_values()
        for identifier, variable in self.parameter_readbacks.items():
            value = actual_parameters.get(identifier)
            variable.set(f"{value:.9g}" if value is not None else "—")
        idle = self._parameter_idle() and not faults and feedback_fresh
        for button in self._idle_buttons + self._parameter_buttons:
            button.configure(state="normal" if idle else "disabled")
        self.align_button.configure(state="normal" if idle and all(status.get(k) for k in ("offset_ready", "sensor_valid", "direction_confirmed")) and not status.get("zero_valid") else "disabled")
        self.clear_button.configure(state="normal" if connected and bool(faults) and not enabled and not busy and not self._closing else "disabled")
        self.mode_selector.configure(state="disabled" if running or busy or self._closing else "readonly")
        missing = []
        if not connected:
            missing.append("连接设备")
        elif not feedback_fresh:
            missing.append("等待状态/诊断反馈")
        else:
            if not session_ready:
                missing.append("STOP 握手")
            if faults:
                missing.append("清除故障")
            required = [("offset_ready", "偏置校准")]
            if self.mode.get() != "open":
                required += [("sense_confirmed", "采样参数"), ("sensor_valid", "有效编码器"),
                             ("direction_confirmed", "方向确认"), ("zero_valid", "转子对齐")]
            missing += [name for key, name in required if not status.get(key)]
        if self.mode.get() == "iq":
            target = _number(self.iq_target.get())
            if target is None:
                missing.append("输入有效 Iq")
            else:
                try:
                    validate_current_target(target, {}, require_readback=False)
                except ValueError as limit_error:
                    missing.append(str(limit_error))
        can_start = not missing and not snapshot.get("tuning_busy") and not busy and not self._closing
        self.start_button.configure(text="更新目标" if running else "启动控制", state="normal" if can_start else "disabled")
        self.start_hint.set("还需：" + "、".join(missing) if missing else "修改输入后点击更新才生效；单次运行最长 60 s。" if running else "修改输入不会发指令；点击启动后开始持续控制。")
        self.phase_text.set("01 连接设备" if not connected else "运行中 · 更新目标 / STOP 停止" if running else "02 检查与校准" if missing else "03 设置目标 → 点击启动")
        self.connect_button.configure(state="disabled" if connected or busy or self._closing else "normal")
        self.disconnect_button.configure(state="normal" if (connected or busy) and not self._closing else "disabled")
        self.recover_button.configure(state="normal" if active_interface == "pcan" and not running and
                                      not enabled and not busy and not self._closing else "disabled")
        for widget in self._connection_widgets:
            widget.configure(state="disabled" if connected or busy or self._closing else
                             "readonly" if isinstance(widget, ttk.Combobox) else "normal")
        self.read_parameters_button.configure(state="normal" if idle else "disabled")
        node_info = snapshot.get('node_info') or {}
        self.node_info_text.set(f"当前 ID {node_info['active_node']} · 保存 ID {node_info['stored_node']} · "
                               + ('存储记录异常，可在单板停机后重新保存' if node_info.get('storage_error') else
                                  '已掉电保存' if node_info.get('stored_valid') else '出厂默认值') if node_info else
                               '尚未读取板上 ID；顶部“连接 ID”只选择通信地址。')
        self.read_node_button.configure(state='normal' if connected and feedback_fresh and not running and not enabled and not busy and not self._closing else 'disabled')
        self.save_node_button.configure(state='normal' if idle and session_ready and self.single_board.get() else 'disabled')
        self.new_node_editor.configure(state='disabled' if busy or self._closing else 'normal')
        self.single_board_check.configure(state='disabled' if busy or self._closing else 'normal')
        calibration_info = snapshot.get('calibration_info') or {}
        if calibration_info.get('storage_error'):
            calibration_text = 'Flash 记录异常：闭环被禁止；停机后清除，或重新完成空载对齐再保存。'
        elif calibration_info.get('profile_mismatch'):
            calibration_text = '保存值与当前电机配置不匹配：闭环被禁止，请清除后按当前电机重新空载对齐。'
        elif calibration_info.get('stored_valid'):
            calibration_text = (f"已保存 · 方向 {calibration_info.get('sensor_direction', 0):+d} · "
                                f"电角零点 {calibration_info.get('electrical_zero_rad', 0):.3f} rad · "
                                + ('本次启动已恢复' if calibration_info.get('runtime_valid') else '本次启动尚未恢复'))
        else:
            calibration_text = '未保存：真正掉电后再次上电会执行一次自动吸合对齐。'
        self.calibration_info_text.set(calibration_text if calibration_info else
            '尚未读取。第一次空载对齐成功后再保存；装车带载时不要重新对齐。')
        calibration_idle = (connected and feedback_fresh and not running and not enabled and
                            not busy and not snapshot.get('tuning_busy') and not self._closing)
        self.read_calibration_button.configure(state='normal' if calibration_idle else 'disabled')
        can_save_calibration = calibration_idle and session_ready and all(status.get(key) for key in
            ('offset_ready', 'sensor_valid', 'direction_confirmed', 'zero_valid'))
        self.save_calibration_button.configure(state='normal' if can_save_calibration else 'disabled')
        self.clear_calibration_button.configure(state='normal' if calibration_idle and session_ready else 'disabled')
        self._refresh_parameter_states()
        stop_text = "STOP 已确认" if snapshot.get("stop_confirmed") else "STOP 尚未确认"
        connection = "反馈正常" if feedback_fresh else "反馈中断" if connected else "未连接"
        self.footer.set(f"{'SIM 模拟器' if simulator else 'CAN 实机'}  /  {connection}  /  {stop_text}  /  "
                        f"TX {snapshot.get('tx_frames', 0):,} · RX {snapshot.get('rx_frames', 0):,}  /  "
                        f"自动恢复 {'开' if snapshot.get('auto_reconnect_enabled') else '关'} · "
                        f"成功 {snapshot.get('recovery_count', 0)} 次  /  CSV {len(self._samples):,} 点")

    @staticmethod
    def _append(widget, text, max_lines):
        at_bottom = widget.yview()[1] >= 0.98
        widget.configure(state="normal")
        widget.insert("end", text.rstrip("\n") + "\n")
        count = int(widget.index("end-1c").split(".")[0])
        if count > max_lines:
            widget.delete("1.0", f"{count - max_lines + 1}.0")
        widget.configure(state="disabled")
        if at_bottom:
            widget.see("end")

    def _log(self, text):
        self._append(self.terminal, f"{datetime.now():%H:%M:%S}  {text}", 1000)

    def _handle_events(self, events):
        frame_lines = []
        log_lines = []
        for event in events:
            kind = event.get("type")
            if kind == 'node_changed':
                self.node.set(str(event['node']))
                self.single_board.set(False)
                self._plot_samples.clear()
                self._plot_source = None
            event_time = event.get("time", "")
            numeric_time = _number(event_time)
            if numeric_time is not None:
                try:
                    event_time = datetime.fromtimestamp(numeric_time).strftime("%H:%M:%S.%f")[:-3]
                except (ValueError, OSError, OverflowError):
                    event_time = str(event_time)
            if kind == "log":
                log_lines.append(f"{event_time}  {event.get('message', '')}")
            elif kind == "sample":
                sample = {key: event.get(key) for key in CSV_FIELDS}
                timestamp = _number(sample.get("timestamp"))
                if timestamp is None:
                    timestamp = time.monotonic()
                    sample["timestamp"] = timestamp
                sample["interface"] = self._snapshot.get("interface") or INTERFACES[self.interface.get()]
                self._samples.append(sample)
                self._plot_samples.append(sample)
            elif kind == "frame":
                identifier = event.get("id", "?")
                if isinstance(identifier, int):
                    identifier = f"0x{identifier:03X}"
                data = event.get("data", "")
                if isinstance(data, (bytes, bytearray, list, tuple)):
                    try:
                        data = " ".join(f"{int(byte):02X}" for byte in data)
                    except (ValueError, TypeError):
                        data = str(data)
                frame_lines.append(f"{event_time}  {str(event.get('direction', '')).upper():>2}  {identifier}  {data}")
        if frame_lines:
            self._append(self.frames, "\n".join(frame_lines[-600:]), 600)
        if log_lines:
            self._append(self.terminal, "\n".join(log_lines[-1000:]), 1000)

    def _draw_plot(self):
        if self._destroying:
            return
        canvas = self.canvas
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 150 or height < 100:
            return
        round_rect(canvas, 0, 0, width, height, radius=18, fill=COLORS['input'], outline='')
        left, top, right, bottom = 62, 22, width-18, height-35
        span = _number(self.plot_span.get(), 10.0)
        latest = self._plot_samples[-1]["timestamp"] if self._plot_samples else 0.0
        samples = [sample for sample in self._plot_samples if latest-span <= sample["timestamp"] <= latest]
        kind = self.plot_kind.get()
        if kind == "角度 / °":
            series = [("angle_deg", COLORS["accent"])]
            low, high = 0, 360
        else:
            series = [("rpm", COLORS["blue"])] if kind == "转速 / rpm" else [(key, color) for key, (visible, _, color) in self.visible_curves.items() if visible.get()]
            values = [abs(value) for sample in samples for key, _ in series for value in [_number(sample.get(key))] if value is not None]
            high = max(10 if kind == "转速 / rpm" else .05, max(values, default=0)*1.15)
            low = -high
        for tick in range(5):
            y = top+(bottom-top)*tick/4
            value = high-(high-low)*tick/4
            canvas.create_line(left, y, right, y, fill=COLORS["grid"])
            canvas.create_text(left-9, y, text=f"{value:.0f}" if kind != "电流 / A" else f"{value:.3f}", anchor="e", fill=COLORS["muted"], font=("Consolas", 9))
        for tick in range(6):
            x = left+(right-left)*tick/5
            canvas.create_text(x, bottom+18, text=f"{-(span*(1-tick/5)):g}s", fill=COLORS["muted"], font=("Consolas", 9))
        for key, color in series:
            points, previous = [], None
            for sample in samples:
                value = _number(sample.get(key))
                wraps = value is not None and previous is not None and key == "angle_deg" and abs(value-previous) > 180
                if value is None or wraps:
                    if len(points) >= 4:
                        canvas.create_line(*points, fill=color, width=2)
                    points = []
                if value is not None:
                    x = left+(sample["timestamp"]-latest+span)/span*(right-left)
                    y = top+(high-value)/(high-low)*(bottom-top)
                    points.extend((x, y))
                previous = value
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2)
        if not samples:
            canvas.create_text((left+right)/2, (top+bottom)/2, text="连接后显示遥测；连接不会启动电机", fill=COLORS["muted"], font=(FONT, 10))

    def _clear_plot(self):
        self._samples.clear()
        self._plot_samples.clear()
        self._draw_plot()

    def _export_csv(self):
        if not self._samples:
            self._show_error("尚无可导出的遥测采样。")
            return
        path = filedialog.asksaveasfilename(parent=self.root, title="导出电流采样 CSV", defaultextension=".csv", filetypes=(("CSV 文件", "*.csv"),), initialfile=f"foc_current_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return
        samples = list(self._samples)

        def write_csv():
            try:
                with open(path, "w", newline="", encoding="utf-8-sig") as stream:
                    writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
                    writer.writeheader()
                    writer.writerows(samples)
                self._io_results.put((True, f"已导出 {len(samples):,} 条采样：{path}"))
            except (OSError, ValueError) as error:
                self._io_results.put((False, f"CSV 导出失败：{error}"))

        threading.Thread(target=write_csv, name="foc-csv-export", daemon=True).start()

    def _poll(self):
        if self._destroying:
            return
        try:
            self._render_snapshot(self.controller.snapshot() or {})
            self._handle_events(self.controller.events() or [])
        except Exception as error:
            message = f"界面读取状态失败：{error}"
            if self._local_error != message:
                self._show_error(message)
        while True:
            try:
                success, message = self._io_results.get_nowait()
            except queue.Empty:
                break
            if success == "profile_loaded":
                profile, path = message
                self._import_profile(profile, path)
            elif success:
                self._log(message)
            else:
                self._show_error(message)
        self._draw_plot()
        if self._closing and (self._snapshot.get("stop_confirmed") or not self._snapshot.get("connected") or time.monotonic() >= self._close_deadline):
            if self._snapshot.get("connected") and not self._snapshot.get("stop_confirmed"):
                self._log("关闭前未收到 STOP 执行确认，不能确认电机已停止。")
            self._invoke("disconnect")
            self._invoke("close")
            self._destroying = True
            self.root.after(200, self.root.destroy)
            return
        self.root.after(50, self._poll)

    def _request_close(self):
        if self._closing:
            return
        self._closing = True
        self._close_deadline = time.monotonic() + 2.0
        self._invoke("stop")
        self._render_snapshot(self._snapshot)


def run_gui():
    """Open the desktop console without issuing any motion command."""
    root = tk.Tk()
    FocDesktopUI(root)
    root.mainloop()


if __name__ == "__main__":
    run_gui()
