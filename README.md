# HAB-RECON — high-altitude balloon reconnaissance (EDTH Sofia)

A balloon payload that flies to 20 km, rides the jet stream to a target area and photographs it,
plus the ground station and a web dashboard. For the demo the flight is simulated, but the
sensors, the radio link and the camera pipeline are real software.

- **Mission (default):** launch from Sofia, ride the jet stream north-east, photograph four places
  on the way (Sofia Airport, Hemus motorway at Vitinya, Lovech, Levski railway junction), climb to
  20 km over **Ruse Airport**, parachute down.
- **Camera:** pictures are made from real satellite imagery (Esri) and run through an object
  detector (cars, trucks, planes, ships, bridges, storage tanks…). Every object gets coordinates.
- **Hardware:** XIAO ESP32-S3 payload + GY-91 + Matek F405-WING (INAV 7.1.2), XIAO ESP32-C6
  ground station, optional ESP8266 relay.
- **Wind:** a built-in example field, or the real **NOAA GFS** forecast (downloaded by the camera
  server). **Swarm** tab: 1–100 balloons through the same wind, with radio mesh and camera coverage.

The dashboard shows **HAB-RECON v3.2** in the top-left corner. If you see anything else, you
have an old copy: pull the latest version (see *Get updates*).

Questions: ask **Stefan** (owner of this repo).

---

## 1. Get the code

**With GitHub Desktop (recommended, so you can get updates):**
1. Install GitHub Desktop from https://desktop.github.com and sign in with your GitHub account.
2. Accept the invitation to this repository (you got it by email).
3. In GitHub Desktop: **File → Clone repository…** → pick **odas-scae** → choose a folder → **Clone**.
   - Windows: use a short path without spaces, e.g. `C:\HAB` → the project is `C:\HAB\odas-scae`
   - Mac: e.g. `~/HAB` → the project is `~/HAB/odas-scae`

**Without Git:** on the repository page click **Code → Download ZIP** and unzip it.
(You won't get updates automatically; download again when Stefan says so.)

In the commands below, replace the folder with yours if it is different.

---

## 2. Run the dashboard (no hardware needed)

### 2.1 Install Python (once)
Install **Python 3.12 or 3.13** from https://www.python.org/downloads/
- **Windows:** on the first installer screen tick **"Add python.exe to PATH"**.
- **Mac:** run the installer; use `python3` in the commands.

### 2.2 Create the Python environment (once)
This downloads about 35 MB of packages into a `.venv` folder inside the project.

**Windows (PowerShell):**
```powershell
cd C:\HAB\odas-scae
python -m venv .venv
.venv\Scripts\python -m pip install -r vision\requirements.txt
```

**Mac (Terminal):**
```bash
cd ~/HAB/odas-scae
python3 -m venv .venv
.venv/bin/python -m pip install -r vision/requirements.txt
```

### 2.3 Start the camera server (every time)
**Windows:**
```powershell
cd C:\HAB\odas-scae\vision
..\.venv\Scripts\python vision_server.py --no-browser
```
(or double-click `vision\start_vision.bat` — it does 2.2 and 2.3 by itself)

**Mac:**
```bash
cd ~/HAB/odas-scae/vision
../.venv/bin/python vision_server.py --no-browser
```

Wait for **`Vision server running - open http://localhost:8765/`**.
**Keep this window open** — closing it (or Ctrl+C) stops the camera.
Use a second window for anything else.

### 2.4 Open the dashboard
Open **http://localhost:8765** in **Chrome or Edge** (not Safari, not Brave — the hardware
connection needs Web Serial). Press **Ctrl+F5** (Mac: **Cmd+Shift+R**) once to load the newest page.
Check: top-left says **HAB-RECON v3.2**, the **Camera · recon** tab shows a green dot.

### 2.5 Fly the demo
1. Press **Fit flight** on the map: the route Sofia → Ruse with the numbered recon stops.
   The **Recon stops** card on the right says **"5 of 5 on the route"**.
2. Press **Demo mode**, then **Launch**. Keep **Demo speed 60×**.
3. As the balloon passes each stop, the camera picture opens by itself (15–30 s per picture).
4. After landing (about 4½ minutes), open the **Intel report** tab: one line per picture,
   **Spreadsheet (CSV)** and **Google Earth (KML)** export.

**Before a demo at a new place (with internet):** run the storyboard once — it takes the
picture of every stop and keeps the satellite imagery, so the demo then works offline.
- Windows: double-click `vision\make_storyboard.bat`
- Mac: `cd ~/HAB/odas-scae/vision && ../.venv/bin/python make_storyboard.py`

It opens `recon_storyboard.html` with the real pictures of all stops.

### 2.6 Change the mission
- **Add stop** on the map → click a place → give it a name. Drag a number to move a stop,
  ✕ in the Recon stops card removes it. The card warns when a stop is more than 15 km from the route.
- **Set target** → click the map. **Default mission** puts Sofia → Ruse back.

### 2.7 Real wind (NOAA GFS forecast)
The wind panel (top left of the map) has two new rows:
- **Data:** *Example field* (built-in, not real weather; the recorded demo uses this) or
  **GFS forecast · NOAA**. With GFS, the camera server downloads NOAA's newest forecast for a
  box of about ±5° around the launch site (28 heights up to 31 km, every 3 h, 48 h ahead;
  ~9 MB, 1–2 minutes the first time). The note under it shows the forecast run.
- **Launch:** the launch time (your local time; UTC next to it). The plan, the demo flight and
  the swarm all start at this time.

The camera server must be running (2.3). Every forecast is kept in `vision/wind_cache`, so it
also works offline later. **Before a demo without internet**, download 5 days ahead once:
- Windows: double-click `vision\download_wind.bat`
- Mac: `cd ~/HAB/odas-scae/vision && ../.venv/bin/python gfs_wind.py --hours 120`

Data: NOAA/NCEP Global Forecast System (GFS) 0.25°, from NOMADS (public, no account).

### 2.8 Swarm simulation (1–100 balloons)
Open the **Swarm** tab at the bottom:
1. **Balloons:** how many (1–100).
2. **Behaviour:**
   - *Float in an altitude band* — the balloons share out the band (e.g. 12–20 km), hold their
     height with the vent valve and ballast, then cut down after the set hours. Different heights
     have different winds, so the swarm spreads out.
   - *Jet-stream mission to the target* — every balloon flies the same mission as the plan.
   - *Free flight* — an ordinary latex balloon: climbs until it bursts.
3. **Launch:** one site one after another (every N minutes), sites along a line (e.g. along the
   front: length and direction), or random sites in a circle.
4. **Spread:** lift ± % (each balloon climbs a bit faster or slower) and wind error in m/s (how
   wrong the forecast may be; it changes slowly, over about 2 hours).
5. **Radio:** range of the balloon-to-balloon and balloon-to-ground radio. A link also needs line
   of sight over the curved Earth.
6. **Run swarm.** Then drag the time slider or press ▶ (1 hour per second).

What you see: every track (colour = balloon number), the balloons at the slider time, radio
links (green = to the ground station at the launch site, blue = between balloons; a red ring =
no link), and with *camera coverage* where the gimbal camera could take pictures (within 15 km,
at least 6 km up). The tiles show how many are flying, linked and watching the target; the chart
shows the same over time; the table has one row per balloon (click a row to follow it).
**CSV** and **KML** (Google Earth, 3-D tracks) export everything.

---

## 3. Record the demo video

1. Do one practice flight first, then reload the page (F5) so everything is back on the pad.
2. Browser full screen (Windows **F11**, Mac **Ctrl+Cmd+F**), turn on Do Not Disturb.
3. Start recording:
   - **Windows:** **Win+Shift+R** (Snipping Tool) → select the screen → **Start**.
     Saved in *Videos → Screen Recordings*.
   - **Mac:** **Cmd+Shift+5** → *Record Entire Screen* → **Record** (Options → Microphone for voice).
     Stop with the ■ button in the menu bar. Saved on the Desktop.
4. What to show (mission clock at the top):

| When | Show |
|---|---|
| before launch | the route, the jet-stream ride, the grey dotted line = same balloon without altitude control (misses by ~146 km), the Recon stops card |
| T+0:20 | Sofia Airport picture opens — point at a box: it shows coordinates |
| T+0:38 / 1:27 / 1:51 | Hemus motorway, Lovech, Levski junction — green ticks in the Recon stops card |
| any time | Sensor health: "7 of 7 agree" (two independent sensor chains) |
| T+3:02 | Ruse Airport from 20 km |
| after landing | Intel report tab → Google Earth (KML) |

Don't click objects in the picture while recording (the map zooms away; press **Fit flight** to come back).
Edit to ~2½ minutes (Windows: Clipchamp, Mac: iMovie).

---

## 4. Hardware

### 4.1 Arduino IDE (once)
1. Install **Arduino IDE 2**.
2. **Tools → Board → Boards Manager** → install **esp32 by Espressif Systems** (3.x).
3. **Tools → Manage Libraries** → install **Adafruit BMP280 Library** (accept "install all").

Board settings (Tools menu):

| Board | Board name in Arduino | Also set | Sketch |
|---|---|---|---|
| Payload | **XIAO_ESP32S3** | USB CDC On Boot: **Enabled** | `payload_s3/payload_s3.ino` |
| Ground station | **XIAO_ESP32C6** | USB CDC On Boot: **Enabled** | `ground_c6/ground_c6.ino` |
| Relay (optional) | LOLIN(WEMOS) D1 R2 & mini / NodeMCU 1.0 | — | `relay_8266/relay_8266.ino` |

Open the `.ino` **from its folder** — the `.h` files next to it must stay in the same folder.
Ports: Windows shows **USB Serial Device (COMx)**, Mac shows **/dev/cu.usbmodem…**.
To see which is which: unplug a board and see which port disappears.

### 4.2 Wiring (payload)

| From | To XIAO S3 |
|---|---|
| GY-91 VIN | 5 V rail (+) |
| GY-91 GND | GND rail (−) |
| GY-91 SCL | D5 |
| GY-91 SDA | D4 |
| F405 TX1 | D0 |
| F405 RX1 | D1 |
| F405 GND | GND rail (−) |

- Clip the **antenna** onto the S3 (small round gold socket). Without it the range is centimetres.
- The **F405** is powered by its own USB (power bank or charger). It already runs INAV 7.1.2 set up
  for MAVLink on UART1 at 115200 — don't change its settings.
- The GY-91 and F405 must sit on one rigid plate, both pointing the same way.

### 4.3 Flash and connect
1. Flash the **C6** with `ground_c6.ino`. Serial Monitor (115200) shows `I,ground station ready on channel 1`.
2. Flash the **S3** with `payload_s3.ino`. Serial Monitor shows `ESP-NOW up on channel 1`
   and a status line every second.
3. Both **yellow lights flicker** (S3 ~5×/s = sending, C6 ~6×/s = receiving).
   The red light is only the charge light — ignore it.
4. **Close Arduino IDE** (it keeps the port busy).
5. Dashboard (Chrome/Edge) → **Connect ground station** → pick the **C6** port → top bar **LINK OK**.
6. Keep everything still → **Calibrate sensors · keep still 20 s** → Sensor health shows **7 of 7 agree**.
7. Compass: the heading comes **only from the GY-91** (the F405 has no compass). Press
   **Calibrate compass** and turn the payload in a figure-8 for 15 s, away from the laptop.
   Without the GY-91 the heading shows "—".

---

## 5. Troubleshooting

| Problem | Fix |
|---|---|
| `http://localhost:8765` "can't be reached" | The server isn't running — do 2.3 again and keep that window open |
| Old dashboard (no **v3.2**) | Pull the latest code (section 6), then Ctrl+F5 / Cmd+Shift+R |
| Brave or Safari opens | Start the server with `--no-browser` and open the address in Chrome/Edge |
| "Failed to open serial port" | Another program has the port: close Arduino IDE, INAV Configurator and other dashboard tabs; unplug and replug the C6 |
| Arduino: "Could not open COMx … doesn't exist" | The port number changed: pick it again in Tools → Port. Still failing: hold the tiny **B** button while plugging the S3 in, upload, then unplug and replug |
| Arduino: `protocol.h: No such file` | The `.ino` was opened outside its folder — open it from `payload_s3/` (or `ground_c6/`) |
| No yellow light on the S3 | The payload program isn't running — open its Serial Monitor, unplug/replug, read the messages |
| S3 flickers, C6 dark | Antenna on the S3, both boards flashed from this repo, 1–3 m apart |
| LINK OK but "No sensor data" | The S3 has old firmware — flash `payload_s3.ino` again |
| Heading differs from a phone | Connect the GY-91 and calibrate the compass (4.3 step 7). The dashboard shows **true** north; a phone set to magnetic north reads about 5.5° less |
| Camera pictures slow | Start the server with `--model n` (about 2× faster, finds fewer small cars) |
| "GFS needs the vision server" | Start the camera server (2.3), then choose *GFS forecast* again |
| GFS: "could not get GFS data" | No internet and nothing in `vision/wind_cache` yet — download once with internet (2.7) |
| "Launch time moved to …" | The kept forecast does not cover your time (offline) — fine for a demo, or download a new one |
| Swarm: many balloons "left the area" | They flew out of the forecast box (±5°). Shorter float time, lower band, or launch nearer the middle |

---

## 6. Get updates / share changes

- **Get the latest:** GitHub Desktop → **Fetch origin** → **Pull origin**.
- **Share a change:** GitHub Desktop lists changed files → write a short summary bottom-left →
  **Commit to main** → **Push origin**.
- After pulling: reload the dashboard (Ctrl+F5); reflash a board only if its `.ino` changed.

---

## 7. Folders

| Folder | What |
|---|---|
| `dashboard/` | the web dashboard (one HTML file) |
| `vision/` | camera server, detector models, storyboard maker, demo route (`demo_route.json`), GFS wind download (`gfs_wind.py`, `download_wind.bat`) |
| `payload_s3/` | payload firmware (XIAO ESP32-S3) |
| `ground_c6/` | ground station firmware (XIAO ESP32-C6) |
| `relay_8266/` | optional relay firmware (ESP8266) |
| `camera_n6/` | script for the real AI camera (not used in the simulated demo) |

**Licence note:** the detector weights (`vision/models`) are AGPL-3.0 and trained on the academic
DOTA dataset — fine for the hackathon, keep this repository **private**, replace before any product.
