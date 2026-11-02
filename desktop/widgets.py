"""Small native Tk dashboard components; no external UI/runtime dependencies."""
import math
import tkinter as tk
from tkinter import ttk

COLORS = dict(bg="#f5f5f2", card="#f5f5f2", input="#ffffff", line="#dfe2dd",
              grid="#edf0eb", text="#282e2b", muted="#78817b", accent="#35736a",
              blue="#527294", purple="#8c7396", green="#35736a", amber="#a17c3e",
              red="#b24c4c")
FONT = "Microsoft YaHei UI"


def configure_theme(root):
    style = ttk.Style(root)
    style.theme_use("clam")
    root.configure(background=COLORS["bg"])
    root.option_add("*TCombobox*Listbox.background", COLORS["input"])
    root.option_add("*TCombobox*Listbox.foreground", COLORS["text"])
    root.option_add("*TCombobox*Listbox.selectBackground", COLORS["line"])
    style.configure(".", background=COLORS["bg"], foreground=COLORS["text"], font=(FONT, 10))
    style.configure("TFrame", background=COLORS["bg"])
    style.configure("Card.TFrame", background=COLORS["card"])
    style.configure("TLabel", background=COLORS["bg"], foreground=COLORS["text"])
    style.configure("Card.TLabel", background=COLORS["card"])
    style.configure("Muted.TLabel", foreground=COLORS["muted"])
    style.configure("Title.TLabel", font=(FONT, 18, "bold"))
    style.configure("Heading.TLabel", font=(FONT, 11, "bold"))
    style.configure("Metric.TLabel", font=("Consolas", 22, "bold"), background=COLORS["card"])
    style.configure("TLabelframe", background=COLORS["bg"], borderwidth=0, bordercolor=COLORS["bg"], relief="flat")
    style.configure("TLabelframe.Label", foreground=COLORS["muted"], font=(FONT, 10, "bold"))
    style.configure("TButton", background=COLORS["bg"], borderwidth=0, padding=(12, 6), focuscolor=COLORS["line"])
    style.map("TButton", background=[("active", "#e6eae3"), ("disabled", COLORS["bg"])],
              foreground=[("disabled", "#afb5af")])
    style.configure("Accent.TButton", background=COLORS["accent"], foreground="white", font=(FONT, 10, "bold"))
    style.map("Accent.TButton", background=[("active", "#46847a"), ("disabled", "#e3e8e0")],
              foreground=[("disabled", "#9ca69d")])
    for name in ("TEntry", "TSpinbox", "TCombobox"):
        style.configure(name, fieldbackground=COLORS["input"], background=COLORS["line"],
                        foreground=COLORS["text"], bordercolor=COLORS["line"], padding=4,
                        arrowcolor=COLORS["muted"], insertcolor=COLORS["text"])
        style.map(name, fieldbackground=[("readonly", COLORS["input"]), ("disabled", "#efefeb")],
                  foreground=[("disabled", "#9ca69d")], selectbackground=[("!disabled", COLORS["line"])])
    style.configure("TNotebook", borderwidth=0, bordercolor=COLORS["bg"], lightcolor=COLORS["bg"],
                    darkcolor=COLORS["bg"], background=COLORS["bg"], tabmargins=(0, 0, 0, 8))
    style.configure("TNotebook.Tab", background=COLORS["bg"], foreground=COLORS["muted"],
                    padding=(18, 10), borderwidth=0, bordercolor=COLORS["bg"],
                    lightcolor=COLORS["bg"], darkcolor=COLORS["bg"])
    style.layout("TNotebook.Tab", [("Notebook.padding", {"sticky": "nswe", "children": [
        ("Notebook.label", {"sticky": "nswe"})]})])
    style.map("TNotebook.Tab", background=[("selected", COLORS["card"])],
              foreground=[("selected", COLORS["accent"])], font=[("selected", (FONT, 10, "bold"))])
    style.configure("TCheckbutton", background=COLORS["bg"], foreground=COLORS["muted"])
    style.map("TCheckbutton", background=[("active", COLORS["bg"])])
    style.configure("Vertical.TScrollbar", background=COLORS["line"], troughcolor=COLORS["bg"],
                    bordercolor=COLORS["bg"], arrowcolor=COLORS["muted"])
    return style


class MetricCard(ttk.Frame):
    def __init__(self, parent, title, variable, accent=None, small=False):
        super().__init__(parent, style="Card.TFrame", padding=(16, 12))
        ttk.Label(self, text=title, style="Card.TLabel", foreground=COLORS["muted"],
                  font=(FONT, 9)).pack(anchor="w")
        ttk.Label(self, textvariable=variable, style="Card.TLabel", foreground=accent or COLORS["text"],
                  font=("Consolas", 13 if small else 22)).pack(anchor="w", pady=(6, 0))


def round_rect(canvas, x0, y0, x1, y1, radius=16, **kwargs):
    r = min(radius, (x1-x0)/2, (y1-y0)/2)
    return canvas.create_polygon(x0+r,y0, x1-r,y0, x1,y0, x1,y0+r,
        x1,y1-r, x1,y1, x1-r,y1, x0+r,y1, x0,y1, x0,y1-r,
        x0,y0+r, x0,y0, smooth=True, splinesteps=24, **kwargs)


class RoundedButton(tk.Canvas):
    """Keyboard-accessible button retaining the view's configure/invoke API."""
    def __init__(self, parent, text, command, *, primary=False, danger=False, width=120):
        super().__init__(parent, width=width, height=40, highlightthickness=0,
                         background=COLORS["bg"], takefocus=True, cursor="hand2")
        self._text, self.command = text, command
        self._disabled = self._hover = False
        self._primary, self._danger = primary, danger
        self.bind("<Configure>", lambda event: self.redraw())
        self.bind("<Enter>", lambda event: self._set_hover(True))
        self.bind("<Leave>", lambda event: self._set_hover(False))
        self.bind("<ButtonRelease-1>", lambda event: self.invoke() if 0 <= event.x < self.winfo_width() and 0 <= event.y < self.winfo_height() else None)
        self.bind("<Return>", lambda event: self.invoke())
        self.bind("<space>", lambda event: self.invoke())
        self.bind("<FocusIn>", lambda event: self.redraw())
        self.bind("<FocusOut>", lambda event: self.redraw())

    def _set_hover(self, hovered):
        self._hover = hovered
        self.redraw()

    def configure(self, cnf=None, **kwargs):
        if 'text' in kwargs:
            self._text = kwargs.pop('text')
        if 'state' in kwargs:
            self._disabled = kwargs.pop('state') == 'disabled'
        if kwargs or cnf:
            super().configure(cnf, **kwargs)
        self.redraw()

    config = configure

    def cget(self, key):
        if key == 'text':
            return self._text
        if key == 'state':
            return 'disabled' if self._disabled else 'normal'
        return super().cget(key)

    def instate(self, states):
        return all(self._disabled if item == 'disabled' else not self._disabled
                   if item == '!disabled' else False for item in states)

    def invoke(self):
        if not self._disabled:
            return self.command()

    def redraw(self):
        self.delete('all')
        w, h = self.winfo_width(), self.winfo_height()
        if self._disabled:
            fill, ink = '#e8ebe5', '#a1aaa2'
        elif self._danger:
            fill, ink = ('#a74444' if self._hover else COLORS['red']), 'white'
        elif self._primary:
            fill, ink = ('#46847a' if self._hover else COLORS['accent']), 'white'
        else:
            fill, ink = ('#e3e8e1' if self._hover else '#ebeee8'), COLORS['text']
        round_rect(self, 1, 1, w-1, h-1, radius=12, fill=fill, outline='')
        # A ttk Combobox owns Tcl-only popup widgets. Tkinter's focus_get()
        # cannot resolve their paths and raises KeyError while the menu is open.
        if self.tk.call('focus', '-displayof', self._w) == self._w:
            round_rect(self, 2, 2, w-2, h-2, radius=11, fill='', outline=COLORS['accent'], width=2)
        self.create_text(w/2,h/2,text=self._text,fill=ink,font=(FONT,10))


class EncoderDial(tk.Canvas):
    def __init__(self, parent):
        super().__init__(parent, width=250, height=260, background=COLORS["card"], highlightthickness=0)
        self.readout = None
        self.bind("<Configure>", lambda event: self.redraw())

    def update_readout(self, readout):
        self.readout = readout
        self.redraw()

    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 40 or h < 40:
            return
        cx, cy = w/2, h/2-2
        radius = min(w/2-34, h/2-30)
        valid = self.readout is not None and self.readout.valid
        ink = COLORS["accent"] if valid else COLORS["muted"]
        self.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline=COLORS["line"], width=5)
        for degrees in range(0, 360, 10):
            angle = math.radians(degrees-90)
            outer = radius+12
            inner = radius+4 if degrees % 30 == 0 else radius+8
            self.create_line(cx+inner*math.cos(angle), cy+inner*math.sin(angle),
                             cx+outer*math.cos(angle), cy+outer*math.sin(angle),
                             fill=COLORS["muted"] if degrees % 30 == 0 else COLORS["line"], width=1)
        for degrees in (0, 90, 180, 270):
            angle = math.radians(degrees-90)
            r = radius+24
            self.create_text(cx+r*math.cos(angle), cy+r*math.sin(angle), text=str(degrees),
                             fill=COLORS["muted"], font=("Consolas", 9))
        if valid:
            degrees = self.readout.angle_deg
            if degrees > .01:
                self.create_arc(cx-radius, cy-radius, cx+radius, cy+radius,
                                start=90, extent=-degrees, style="arc", outline=ink, width=5)
            a = math.radians(degrees-90)
            self.create_oval(cx+radius*math.cos(a)-5, cy+radius*math.sin(a)-5,
                             cx+radius*math.cos(a)+5, cy+radius*math.sin(a)+5,
                             fill=COLORS["text"], outline=COLORS["card"], width=2)
        self.create_text(cx, cy-10, text=f"{self.readout.angle_deg:.2f}°" if valid else "—",
                         fill=COLORS['text'] if valid else ink, font=("Consolas", 26))
        self.create_text(cx, cy+23, text="单圈机械角度" if valid else "等待有效角度",
                         fill=COLORS["muted"], font=(FONT, 9))
