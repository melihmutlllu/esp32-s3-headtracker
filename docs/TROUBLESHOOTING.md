# Troubleshooting

## OpenTrack Does Not See Data

Check the firmware serial output:

```text
wifi_status=3
local_ip=...
udp_target=PC_IP:4242
```

Then verify:

- The PC IPv4 address in `secrets.h` matches the current Windows WiFi IP.
- OpenTrack input is set to UDP on port `4242`.
- Windows firewall allows OpenTrack or the UDP listener.
- The ESP32 and PC are on the same network.

PC IP can change with DHCP. The clean fix is a DHCP reservation in the router.

## Test UDP Without OpenTrack

Close OpenTrack first, because only one program should bind port `4242`.

Python:

```powershell
python tools/udp_pose_receiver.py
```

PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File tools/listen_udp_4242.ps1
```

## Yaw Drift

Gyro-only yaw has no absolute reference, so it will drift. The firmware reduces drift with startup calibration and slow stationary gyro bias updates, but a working magnetometer or another external reference is required for true yaw correction.

If drift gets worse as the board warms:

1. Power the tracker.
2. Keep it completely still through startup calibration.
3. Let it sit still for another 30-60 seconds.
4. Recenter in OpenTrack.

If needed, send serial command:

```text
b
```

after the board is warm and completely still.

## Magnetometer Not Found

Run:

```text
firmware/diagnostics/mpu9250_deep_probe/mpu9250_deep_probe.ino
```

Expected for a working MPU9250/AK8963:

```text
i2c_device=0x68
i2c_device=0x0C
ak8963_wia=0x48
```

If `0x0C` never appears, the magnetometer is not accessible.
