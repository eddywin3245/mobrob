#!/usr/bin/env python3
"""
Diagnostic script to test OAK-D device accessibility and firmware state.
"""
import depthai as dai
import sys

def test_device_connection():
    """Test if we can enumerate and connect to OAK device."""
    print("=== OAK-D Device Diagnostic ===\n")

    # List available devices
    print("1. Enumerating available OAK devices...")
    try:
        device_infos = dai.Device.getAllAvailableDevices()
        print(f"   Found {len(device_infos)} device(s)")
        for info in device_infos:
            print(f"   - {info.mxid} (state: {info.state})")
    except Exception as e:
        print(f"   ERROR: {e}")
        return False

    if not device_infos:
        print("   No devices found!")
        return False

    # Try to connect to first device
    print("\n2. Attempting connection to first device...")
    try:
        with dai.Device() as device:
            print(f"   Connected successfully!")
            print(f"   Device name: {device.getDeviceName()}")
            print(f"   Firmware version: {device.getFirmwareVersion()}")
            print(f"   Device state: OK")
        return True
    except Exception as e:
        print(f"   ERROR: {e}")
        print(f"   Error type: {type(e).__name__}")
        return False

if __name__ == '__main__':
    success = test_device_connection()
    sys.exit(0 if success else 1)
