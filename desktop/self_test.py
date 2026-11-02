"""Reproducible packaged-app check. Always uses the in-process simulator."""
import argparse
import json
from pathlib import Path
import time

from .controller import Controller
from .current_limits import HOST_CURRENT_LIMIT_A
from .profiles import validate_profile
from .telemetry import encoder_readout


def run(report_path):
    controller = Controller()
    frames = samples = 0
    transmitted_iq = []

    def wait(check, timeout=3):
        nonlocal frames, samples
        ends = time.monotonic() + timeout
        while time.monotonic() < ends:
            state = controller.snapshot()
            for event in controller.events():
                frames += event["type"] == "frame"
                samples += event["type"] == "sample"
                if event['type'] == 'frame' and event.get('direction') == 'TX' and bytes.fromhex(event['data'])[1] == 1:
                    transmitted_iq.append(event)
            if check(state):
                return state
            time.sleep(.005)
        raise RuntimeError("模拟流程超时：" + str(controller.snapshot()))

    report = {"simulation_only": True, "passed": False}
    try:
        controller.connect({"interface": "simulator", "node": 1, "bitrate": 1000000,
                            "iq_limit_max_a": 3.3})
        wait(lambda s: s["session_ready"] and s["status"].get("offset_ready"))
        controller.read_node()
        wait(lambda s: s['node_info'].get('active_node') == 1)
        controller.change_node(3, single_board=True)
        wait(lambda s: s.get('node') == 3 and s['session_ready'] and not s['node_changing'] and
             s['node_info'].get('stored_valid') and s['status'].get('offset_ready'))
        controller.read_parameters()
        wait(lambda s: len(s["parameters"]) == 9 and bool(s.get('limits_info')))
        plan = validate_profile({"schema_version": 1,
                                 "parameters": {"current_kp": .8, "current_ki": 500}})
        restored = validate_profile(json.loads(json.dumps(plan.to_dict())))
        controller.set_parameters(restored.to_mapping())
        wait(lambda s: abs(s["parameters"].get(1, 0) - .8) < 1e-6 and
             s["parameters"].get(2) == 500 and not s["tuning_busy"])
        controller.submit("dir 1")
        wait(lambda s: s["status"].get("direction_confirmed"))
        controller.submit("align")
        wait(lambda s: s["status"].get("zero_valid") and not s["running"])
        iq_before = len(transmitted_iq)
        controller.submit("iq 3.3")
        rejected = wait(lambda s: '电流限幅' in s.get('error', '') and not s['running'] and s['stop_confirmed'])
        if len(transmitted_iq) != iq_before:
            raise RuntimeError('读回1A时3.3A请求不应发送')
        controller.set_parameter(3, 3.3)
        raised = wait(lambda s: abs(s['parameters'].get(3, 0) - 3.3) < 1e-6)
        if raised['limits_info'].get('phase_trip_a') != 2.0:
            raise RuntimeError('独立相电流保护未正确读回')
        controller.submit("iq .1")
        positive = wait(lambda s: s["status"].get("iq_a", 0) >= .09)
        encoder = encoder_readout(positive)
        if not encoder.valid:
            raise RuntimeError("模拟编码器角度未有效回传")
        controller.submit("iq -.1")
        negative = wait(lambda s: s["status"].get("iq_a", 0) <= -.09)
        controller.stop()
        stopped = wait(lambda s: s["stop_confirmed"] and not s["status"].get("driver_enabled"))
        report.update(passed=True, profile_round_trip=True, batch_confirmed=True,
                      host_input_max_a=HOST_CURRENT_LIMIT_A,
                      initial_iq_limit_a=rejected['parameters'][3],
                      board_iq_limit_a=stopped['parameters'][3],
                      request_cap_a=stopped['limits_info']['request_max_a'],
                      phase_trip_a=stopped['limits_info']['phase_trip_a'],
                      request_limit_3p3_written_and_read_back=True,
                      high_target_rejected_by_readback_before_tx=True,
                      node_config_saved=True, active_node=stopped['node'], stored_node=stopped['node_info']['stored_node'],
                      encoder_feedback=True, angle_deg=encoder.angle_deg,
                      positive_iq_a=positive["status"]["iq_a"],
                      negative_iq_a=negative["status"]["iq_a"], stop_confirmed=True,
                      parameters=stopped["parameters"], frames=frames, samples=samples)
    except Exception as error:
        report["error"] = str(error)
    finally:
        controller.close()
        report["worker_closed"] = controller.wait_closed(1)
        if not report["worker_closed"]:
            report["passed"] = False
        Path(report_path).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if report["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    return run(args.report)
