#!/usr/bin/env python3
"""
Pushes config.yaml to the robot via ADB so it takes effect on next OpMode init,
without requiring a full APK redeploy.

Usage:
  python3 scripts/push_config.py            # push
  python3 scripts/push_config.py --reset    # remove override, revert to bundled config
"""
import os
import sys
import subprocess

BASE_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOCAL_YAML  = os.path.join(BASE_DIR, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml")
REMOTE_DIR  = "/sdcard/FIRST/teamcode"
REMOTE_YAML = f"{REMOTE_DIR}/config.yaml"


def run(cmd, check=True):
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and result.returncode != 0:
        print(f"ERROR: {result.stderr.strip() or result.stdout.strip()}")
        sys.exit(1)
    return result


def get_device():
    result = run("adb devices", check=False)
    lines = [l.strip() for l in result.stdout.splitlines() if l.strip() and "List of devices" not in l]
    devices = [l.split()[0] for l in lines if "device" in l and "offline" not in l]
    if not devices:
        print("ERROR: No ADB device found. Is the Control Hub connected and ADB enabled?")
        sys.exit(1)
    if len(devices) > 1:
        print(f"WARNING: Multiple devices found, using first: {devices[0]}")
    return devices[0]


def push_config(device):
    print(f"Pushing config.yaml → robot ({device})")
    run(f"adb -s {device} shell mkdir -p {REMOTE_DIR}", check=False)
    run(f'adb -s {device} push "{LOCAL_YAML}" {REMOTE_YAML}')
    print(f"✓ Pushed to {REMOTE_YAML}")
    print("  Re-init your OpMode on the robot to pick up the new values.")


def reset_config(device):
    print(f"Removing config override from robot ({device})")
    result = run(f"adb -s {device} shell rm -f {REMOTE_YAML}", check=False)
    if result.returncode == 0:
        print(f"✓ Removed {REMOTE_YAML} — robot will use bundled config on next init.")
    else:
        print("  No override file found (already using bundled config).")


def main():
    device = get_device()
    if "--reset" in sys.argv:
        reset_config(device)
    else:
        push_config(device)


if __name__ == "__main__":
    main()
