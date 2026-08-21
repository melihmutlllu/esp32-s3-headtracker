#pragma once

// Copy this file as secrets.h in the same folder, then fill your local values.
// secrets.h is intentionally ignored by git.

#define HEADTRACKER_WIFI_SSID "your-wifi-name"
#define HEADTRACKER_WIFI_PASSWORD "your-wifi-password"

// Use the IPv4 address of the PC running OpenTrack.
// Example: Windows Wi-Fi IP 192.168.1.3 => 192, 168, 1, 3
#define HEADTRACKER_OPENTRACK_PC_IP 192, 168, 1, 3

// Optional AP fallback credentials if you enable ENABLE_AP_FALLBACK in the sketch.
#define HEADTRACKER_AP_SSID "ESP32-HeadTracker"
#define HEADTRACKER_AP_PASSWORD "headtrack123"
