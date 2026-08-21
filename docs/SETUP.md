# Setup

## Arduino

Tested with Arduino IDE and the ESP32 board package.

Board/FQBN:

```powershell
esp32:esp32:lolin_s3_mini:CDCOnBoot=cdc
```

Compile with Arduino CLI:

```powershell
arduino-cli compile --fqbn esp32:esp32:lolin_s3_mini:CDCOnBoot=cdc firmware/headtracker_udp_lowpower
```

Upload example:

```powershell
arduino-cli upload -p COM23 --fqbn esp32:esp32:lolin_s3_mini:CDCOnBoot=cdc firmware/headtracker_udp_lowpower
```

## Local Secrets

Create:

```text
firmware/headtracker_udp_lowpower/secrets.h
```

from:

```text
firmware/headtracker_udp_lowpower/secrets.example.h
```

Example:

```cpp
#pragma once

#define HEADTRACKER_WIFI_SSID "your-wifi-name"
#define HEADTRACKER_WIFI_PASSWORD "your-wifi-password"
#define HEADTRACKER_OPENTRACK_PC_IP 192, 168, 1, 3
```

To find the PC IP on Windows:

```powershell
Get-NetIPAddress -AddressFamily IPv4
```

Use the IPv4 address of the WiFi adapter that is on the same network as the ESP32.

## OpenTrack

- Input: UDP over network
- Port: `4242`
- Output: FreeTrack 2.0 Enhanced, or TrackIR if a game needs it
- Filter: Accela is a good starting point

OpenTrack centering does not update the ESP32's internal center. For quick use this is fine, but for a more polished build add a physical recenter button or send the serial `c` command to the ESP32.

## Startup Calibration

Keep the tracker still after power-up. The firmware waits for WiFi, lets the board settle, and then collects gyro/accel calibration samples.

If the first calibration was noisy, keep it still and send:

```text
b
```

from Serial Monitor.
