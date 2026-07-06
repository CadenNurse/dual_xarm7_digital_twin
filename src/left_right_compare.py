#!/usr/bin/env python3
from xarm.wrapper import XArmAPI
import time
import json

LEFT_IP = "192.168.1.211"
RIGHT_IP = "192.168.1.221"

def safe_call(label, fn):
    try:
        value = fn()
        return {"ok": True, "value": value}
    except Exception as e:
        return {"ok": False, "value": f"{type(e).__name__}: {e}"}

def connect_arm(ip):
    arm = XArmAPI(ip, is_radian=False)
    time.sleep(1.0)
    return arm

def snapshot(name, arm):
    data = {"name": name}

    data["state"] = safe_call("state", lambda: arm.get_state())
    data["err_warn"] = safe_call("err_warn", lambda: arm.get_err_warn_code())
    data["cmdnum"] = safe_call("cmdnum", lambda: arm.get_cmdnum())
    data["version"] = safe_call("version", lambda: arm.version)
    data["angles"] = safe_call("angles", lambda: arm.angles)
    data["position"] = safe_call("position", lambda: arm.position)
    data["last_used_tcp_speed"] = safe_call("last_used_tcp_speed", lambda: arm.last_used_tcp_speed)
    data["last_used_joint_speed"] = safe_call("last_used_joint_speed", lambda: arm.last_used_joint_speed)
    data["last_used_tcp_acc"] = safe_call("last_used_tcp_acc", lambda: arm.last_used_tcp_acc)
    data["last_used_joint_acc"] = safe_call("last_used_joint_acc", lambda: arm.last_used_joint_acc)
    data["mode"] = safe_call("mode", lambda: arm.mode)
    data["has_err_warn"] = safe_call("has_err_warn", lambda: arm.has_err_warn)
    data["is_connected"] = safe_call("is_connected", lambda: arm.connected)
    data["is_ready"] = safe_call("is_ready", lambda: arm.ready)

    # Optional API calls, depending on SDK/firmware support
    optional_calls = {
        "tcp_load": lambda: arm.get_tcp_load(),
        "tcp_offset": lambda: arm.get_tcp_offset(),
        "world_offset": lambda: arm.get_world_offset(),
        "mount_direction": lambda: arm.get_mount_direction(),
        "gravity_direction": lambda: arm.get_gravity_direction(),
        "reduced_mode": lambda: arm.get_reduced_mode(),
        "reduced_states": lambda: arm.get_reduced_states(),
        "self_collision_params": lambda: arm.get_self_collision_detection(),
        "collision_sensitivity": lambda: arm.get_collision_sensitivity(),
        "teach_sensitivity": lambda: arm.get_teach_sensitivity(),
    }

    for key, fn in optional_calls.items():
        data[key] = safe_call(key, fn)

    return data

def fmt(v):
    if isinstance(v, float):
        return round(v, 4)
    if isinstance(v, (list, tuple)):
        return [fmt(x) for x in v]
    return v

def compare_dicts(left, right):
    print("\n" + "=" * 80)
    print("LEFT vs RIGHT")
    print("=" * 80)

    keys = sorted(set(left.keys()) | set(right.keys()))
    for key in keys:
        if key == "name":
            continue
        l = left.get(key)
        r = right.get(key)

        lv = fmt(l["value"]) if isinstance(l, dict) and "value" in l else fmt(l)
        rv = fmt(r["value"]) if isinstance(r, dict) and "value" in r else fmt(r)

        same = lv == rv
        status = "SAME " if same else "DIFF "
        print(f"\n[{status}] {key}")
        print(f"  LEFT : {lv}")
        print(f"  RIGHT: {rv}")

def main():
    print("Connecting to LEFT arm...")
    left = connect_arm(LEFT_IP)

    print("Connecting to RIGHT arm...")
    right = connect_arm(RIGHT_IP)

    time.sleep(1.0)

    print("Collecting LEFT snapshot...")
    left_data = snapshot("left", left)

    print("Collecting RIGHT snapshot...")
    right_data = snapshot("right", right)

    print("\nRAW JSON")
    print(json.dumps({"left": left_data, "right": right_data}, indent=2, default=str))

    compare_dicts(left_data, right_data)

    print("\nDone.")

    try:
        left.disconnect()
    except Exception:
        pass
    try:
        right.disconnect()
    except Exception:
        pass

if __name__ == "__main__":
    main()