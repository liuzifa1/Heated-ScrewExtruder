# Heated Screw Extruder Firmware

ESP32-S3 firmware for a heated screw extruder / composites mixer controller.
It provides:

- Motor control over RS-485 Modbus (AQMD22A04BLS-Ex driver)
- Heater control through a solid-state relay (SSR)
- Temperature sensing with PT100 + MAX31865
- RTC timekeeping through PCF8563
- A Wi-Fi AP + Telnet command interface for live operation

The default firmware entrypoint is `src/main.c` (production).  
An optional Wi-Fi scan demo firmware exists in `src/wifi_scan_debug.c`.

---

## 1) Hardware and Interfaces

### MCU target
- ESP32-S3 (16 MB flash / 8 MB PSRAM board profile in `platformio.ini`)

### Peripheral map (from firmware defaults)
- **RS-485 UART1** (motor driver): TX `GPIO13`, RX `GPIO8`
- **MAX31865 SPI2** (PT100): CLK `GPIO2`, MISO `GPIO39`, MOSI `GPIO41`, CS `GPIO36`
- **PCF8563 I2C0** (RTC): SCL `GPIO11`, SDA `GPIO9`
- **Heater SSR control**: `GPIO5`

### Controlled / monitored devices
- Brushless motor driver: AQMD22A04BLS-Ex (Modbus RTU)
- Heating belt via SSR
- PT100 RTD via MAX31865
- RTC via PCF8563

---

## 2) Main Runtime Behavior

At startup (`app_main`), firmware initializes:

1. NVS and previously saved runtime settings
2. Wi-Fi SoftAP
3. RS-485 (motor driver channel)
4. SSR output
5. RTC
6. MAX31865 temperature frontend
7. Background tasks:
   - Telnet server
   - Temperature / heater control loop
   - Motor telemetry polling
   - Heartbeat logging

### Key control capabilities
- Motor start/stop and duty command (`-100%` to `+100%`)
- Auto/manual heater mode
- Target temperature + hysteresis band
- Temperature offset correction
- Material wait/run profiles (named slots)
- Quiet thermal sampling model to reduce motor-noise influence
- Driver telemetry fallback and pause tuning when offline/noisy

### Persistent settings (NVS)
Saved parameters include duty, target temperature, hysteresis band, temp offset,
quiet-sampler parameters, driver quiet-poll parameters, and wait profile settings.

---

## 3) Repository Layout

```text
.
├── platformio.ini                 # PlatformIO environments
├── partitions_16MB.csv            # Flash partition layout
├── src/
│   ├── main.c                     # Production extruder firmware
│   ├── wifi_scan_debug.c          # Wi-Fi scan debug example
│   └── CMakeLists.txt
├── boards/
│   └── esp32-s3-devkitc-1-n16r8.json
├── sdkconfig.production
├── sdkconfig.wifi_scan_debug
└── CMakeLists.txt
```

---

## 4) Build Environment

This project uses **PlatformIO + ESP-IDF** (`framework = espidf`).

### `platformio.ini` environments
- `production` (default): builds `src/main.c`
- `wifi_scan_debug`: builds `src/wifi_scan_debug.c`

Source selection is handled in `src/CMakeLists.txt` using `PIOENV`.

---

## 5) Build / Flash / Monitor

> Run commands from repository root.

### Build production firmware
```bash
pio run -e production
```

### Flash production firmware
```bash
pio run -e production -t upload
```

### Open serial monitor
```bash
pio device monitor -b 115200
```

### Build Wi-Fi scan debug firmware
```bash
pio run -e wifi_scan_debug
```

If your toolchain is not installed, install PlatformIO first:
```bash
pip install platformio
```

---

## 6) Network and Remote Control

Production firmware starts a Wi-Fi access point with defaults:

- SSID: `ScrewExtruders-Heated`
- Password: `12345678`
- Channel: `6`
- Telnet port: `23`

Connect to this AP, then open a telnet session to the device IP and issue commands.

---

## 7) Telnet Command Reference (Production)

| Command | Purpose |
|---|---|
| `allon` / `alloff` | Start motor + heater-auto session / stop all |
| `motoron` / `motoroff` | Start/stop motor only |
| `setduty <-100..100>` | Set motor duty percentage |
| `heaton` / `heatoff` | Manual heater on/off |
| `heatauto` / `heatmanual` | Switch heater control mode |
| `settemp <C>` | Set target temperature and enable AUTO |
| `settempband <C>` | Set temperature hysteresis half-width |
| `settempoffset <C>` | Apply RTD offset correction |
| `profiles` | List material profiles |
| `setprofile <slot> <name> <wait_s> <run_s> <target_C>` | Configure + activate profile slot |
| `useprofile <slot\|name\|none>` | Select active profile |
| `driverquiet` | Show driver quiet-poll settings |
| `setdriverquiet <offline_s> <pause_s>` | Tune driver quiet-poll |
| `setquiet <warm> <cool> <step> <pause_s>` | Tune quiet thermal model |
| `rtdreset` | Force MAX31865 recovery |
| `ls` | Print full system status snapshot |
| `top` | Stream status repeatedly |
| `time` | Read RTC time |
| `settime YYYY-MM-DD HH:MM:SS` | Set RTC/system time |
| `help` | Show command help |

Aliases like `allstart/allstop`, `motorstart/motorstop`, `status/log`, `dash`
are also supported in firmware.

---

## 8) Safety and Operational Notes

- Heater hard cutoff is enforced in firmware (`HEATER_MAX_TEMP_C`).
- If RTD data is invalid/faulted, heater control is constrained for safety.
- `alloff` turns heater off and attempts motor stop.
- Verify Modbus wiring, driver address, and DIP communication mode before operation.
- Tune target temperatures and profiles conservatively during first deployment.

---

## 9) Troubleshooting

- **No build command available:** install PlatformIO (`pip install platformio`).
- **No motor telemetry:** check RS-485 wiring, baud/parity defaults, and driver DIP settings.
- **Frequent RTD faults:** check MAX31865 wiring/shielding and run `rtdreset`.
- **Cannot connect by telnet:** ensure client is on device AP and using port `23`.

---

## 10) License

This project is licensed under the terms in [`LICENSE`](LICENSE).
