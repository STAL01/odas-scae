# HAB-RECON - notes for Claude Code

High-altitude balloon reconnaissance proof of concept for the EDTH Sofia defence
hackathon. Real target: 20 km altitude. For the demo the flight is simulated;
sensors, radio link and the camera pipeline are real software.
The team works on **Windows** (VS Code, PowerShell, Arduino IDE 2, Chrome/Edge).

## Folders
- `payload_s3/` - XIAO ESP32-S3 payload (Arduino). GY-91 (MPU-9250 + BMP280) on I2C
  SDA=D4 SCL=D5; Matek F405-WING V2 over MAVLink on D0 (RX, from F405 TX1) / D1 (TX);
  camera UART D7/D6. Simulated 20 km flight, ESP-NOW telemetry 5 Hz + sensor-health 1 Hz.
  Compares GY-91 with the F405 (bias, noise, live error, WARN 3 sigma / FAULT 6 sigma).
- `ground_c6/` - XIAO ESP32-C6 ground station: ESP-NOW <-> USB serial (115200) for the dashboard.
- `relay_8266/` - optional ESP8266 relay (second hop).
- `dashboard/dashboard.html` - single-file web dashboard (Web Serial, Leaflet map). Contains:
  telemetry/health parsing, the balloon physics (standard atmosphere, wind matrix 31x31x31x13,
  equation of motion with buoyancy, drag, added mass, vent valve + ballast control),
  the jet-stream mission planner, demo mode, recon stops (default mission Sofia -> Ruse Airport,
  `RECON` near the top of the script; gimbal capture within 15 km / above 6 km, `gimbalWatch`
  is shared by the plan and the live flight), the "Camera · recon" tab (rotated boxes by
  category) and the "Intel report" tab (CSV + KML export). Stops are kept in localStorage.
- `vision/` - Python vision server (port 8765): makes the simulated balloon-camera picture from
  Esri World Imagery tiles and runs the YOLO26-OBB detector (ONNX, onnxruntime). `search` param:
  look around N picture widths and frame the busiest spot (`best_frame`, shared with make_storyboard.py).
- `camera_n6/` - untested OpenMV script for the real AI camera (not used in the simulated demo).

## Hard facts - don't change without asking
- `protocol.h` must stay byte-identical in payload_s3, ground_c6 and relay_8266, and match
  `parseTelemetry/encodeTelemetry/parseHealth/encodeHealth` in dashboard.html.
- ESP-NOW channel 1. Telemetry packet 60 bytes, health packet 189 bytes.
- The F405 runs **INAV 7.1.2** (not ArduPilot): UART1 = MAVLink telemetry 115200 only,
  `feature TELEMETRY`. INAV sends no raw accel and no pressure; the payload converts INAV's
  altitude back to pressure and reads gyro from ATTITUDE (deg/s, forward-left-up axes).
- F405 is powered by USB (no LiPo, no GPS, no servo). The GY-91 may be mounted chip-down;
  the firmware detects that at boot.
- Arduino: esp32 core 3.x, boards XIAO_ESP32S3 / XIAO_ESP32C6, "USB CDC On Boot: Enabled",
  library "Adafruit BMP280". Only one program can open a COM port: close the Arduino Serial
  Monitor before connecting the dashboard to the same board.
- Vision model weights are AGPL-3.0 and trained on DOTA (academic): hackathon only.

## Run things (Windows)
- Python packages live in a virtual environment at `hab_recon\.venv` (never install into the
  system Python). Create it once from the hab_recon folder:
  `py -m venv .venv` then `.venv\Scripts\python -m pip install -r vision\requirements.txt`.
  Always run Python as `.venv\Scripts\python` (no activation needed - PowerShell may block
  Activate.ps1 by execution policy).
- Vision server: `cd vision` then `..\.venv\Scripts\python vision_server.py`
  (or double-click `vision\start_vision.bat`, which creates the venv by itself, or the VS Code
  task "Vision: start server"). It serves the dashboard at http://localhost:8765.
  Health check: `curl.exe http://localhost:8765/api/health`.
  Test capture over Sofia: `curl.exe "http://localhost:8765/api/capture?lat=42.6977&lon=23.3219&agl=19400"`
  (JSON with `detections`). Offline test: `..\.venv\Scripts\python vision_server.py --mock-image some.jpg --mock-gsd 0.3`.
- Recon-demo storyboard (real satellite pictures of the planned stops): double-click
  `vision\make_storyboard.bat` or `cd vision` then `..\.venv\Scripts\python make_storyboard.py`.
  Route and stops (Sofia -> Ruse: Sofia Airport, Hemus motorway at Vitinya, Lovech, Levski junction,
  Ruse Airport) are in `vision/demo_route.json`; move a stop by editing its lat/lon and rerun with
  `--only <n>`. Writes `vision/recon_storyboard.html` from `vision/storyboard_template.html`.
- Dashboard alone: open `dashboard\dashboard.html` in Chrome or Edge; "Demo mode" needs no hardware.
- Firmware: Arduino IDE 2 (open the .ino, pick the board and COM port, Upload).

## Style
- Keep the dashboard a single HTML file with no build step.
- Plain, short user-facing text; units in SI.
