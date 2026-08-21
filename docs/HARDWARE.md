# Hardware Notes

## Tested Wiring

ESP32-S3 Mini to IMU:

```text
ESP32-S3 3V3  -> IMU VCC
ESP32-S3 GND  -> IMU GND
ESP32-S3 GPIO8 -> IMU SDA
ESP32-S3 GPIO9 -> IMU SCL
```

The firmware configures I2C at `400 kHz`.

## Sensor Orientation

The current firmware maps the mounted sensor frame like this:

```text
logical X = sensor Y
logical Y = sensor X
logical Z = -sensor Z
```

The output signs are currently tuned so that right yaw and up pitch increase in OpenTrack.

## Power

WiFi bursts can expose weak 3.3 V supplies. For stable behavior:

- Use a clean 3.3 V rail.
- Budget at least 500 mA for the ESP32 board.
- Add local decoupling near the ESP32/IMU.
- A practical starting point is 470 uF bulk plus 10 uF and 100 nF close to the board.

If the board heats excessively from USB, test with a stable external 3.3 V supply and keep WiFi TX power low.

## Magnetometer Reality Check

For a real MPU9250 with AK8963 magnetometer, the deep probe should find:

```text
i2c_device=0x68
i2c_device=0x0C
ak8963_ack=true
ak8963_wia=0x48
```

If only `0x68` appears after bypass, the module behaves like a 6-axis MPU6500-style board or has a disconnected/faulty magnetometer.

SDO/AD0 only changes the main MPU address between `0x68` and `0x69`; it does not make the AK8963 magnetometer appear.
