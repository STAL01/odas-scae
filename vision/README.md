# HAB-RECON vision server

Simulated balloon camera + vehicle detection. The balloon is simulated, so the
camera is too: the server makes the picture the payload camera would take from
real satellite imagery (Esri World Imagery) at the balloon's position, altitude
and heading, adds 20 km of haze, blur and sensor noise, and runs a vehicle
detector on it. Every vehicle comes back with its latitude and longitude.

## Start on Windows

1. Install **Python 3.12 or 3.13** from https://www.python.org/downloads/windows/
   (64-bit installer). On the first screen tick **"Add python.exe to PATH"**.
2. Unzip `hab_recon.zip` (right-click, Extract All) - it does not run from
   inside the zip.
3. Open `hab_recon\vision` and double-click **`start_vision.bat`**.
   If Windows says "Windows protected your PC": More info, then Run anyway.
   The first start installs onnxruntime, numpy and pillow (about 35 MB).
4. The dashboard opens in your browser at http://localhost:8765.
   Use Chrome or Edge. Keep the black window open; closing it stops the server.

`start_vision.bat` keeps everything in a virtual environment, `hab_recon\.venv`:
the first start creates it and installs the packages there, later starts reuse it.

The same by hand, in a terminal (VS Code: Terminal -> New Terminal) in the hab_recon folder:

    py -m venv .venv
    .venv\Scripts\python -m pip install -r vision\requirements.txt
    cd vision
    ..\.venv\Scripts\python vision_server.py

Optional: activate the venv so plain `python` means the venv one -
PowerShell `.venv\Scripts\Activate.ps1`, Command Prompt `.venv\Scripts\activate.bat`.
If PowerShell says "running scripts is disabled", run once:
`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` and answer Y.

The dashboard also works opened as a file (double-click `dashboard.html`) - it
finds the server on port 8765 either way.

## In the dashboard: recon stops, tab "Camera · recon", tab "Intel report"

- **Recon stops** (map and the "Recon stops" card): the default mission is Sofia -> Ruse
  Airport with four stops on the way (Sofia Airport, Hemus motorway at Vitinya, Lovech,
  Levski railway junction). The card shows when the camera passes each stop, from how far,
  and at what resolution, and warns when a stop is out of reach. **Add stop** on the map
  adds one, drag a number to move it, ✕ removes it, **Default mission** restores the demo.
- **Gimbal camera**: in flight (demo or real), when the balloon passes within 15 km of a stop
  and is at least 6 km up, the camera turns to it at the closest point and takes the picture
  (the target: at 20 km). The operator looks around the stop (2-3 picture widths) and
  centres the frame on the busiest spot; the panel says how far that moved it.
- **Camera · recon**: rotated boxes by kind (vehicles, infrastructure, aircraft & ships;
  sports fields and pools hidden by default), overlay with time, altitude, range and
  resolution, scale bar and north arrow. **Picture** picks any earlier picture.
- **Capture** - picture from the balloon (when it flies above 0.5 km), otherwise from 20 km
  above the target. **Aim at map…** - click any spot; the camera looks down at it from 20 km.
  **every 10 s** - a picture straight down every 10 s while the balloon is above 8 km.
- **Lens** - 250 mm (0.27 m per pixel from 20 km: cars are 16 px long) vs 35 mm (1.9 m per
  pixel: cars are 2 px, nothing is found). This is the argument for a telephoto recon camera.
- **Own image…** - run the detector on any satellite picture you have.
- **Intel report**: one line per picture. **Spreadsheet (CSV)** has every detection with its
  position; **Google Earth (KML)** has the picture footprints and the shown detections pinned.
- Scroll on the picture to zoom, drag to pan, double-click to fit. Click an object to fly
  the map to it.
- API for the stops: `/api/capture?lat&lon&agl&search=3` (agl = slant range for a gimbal shot).

## Options (from Command Prompt in the vision folder; `py vision_server.py ...` works the same)

    start_vision.bat --model n                 # ~2x faster, misses more small cars
    start_vision.bat --conf 0.4                # fewer, surer detections
    start_vision.bat --focal 300               # camera lens, mm (default 250)
    start_vision.bat --mock-image photo.jpg --mock-gsd 0.3
                                               # offline: your own north-up image, 0.3 m/px

Satellite tiles are cached in `tile_cache/`, so places you have looked at once
also work without internet. Open the dashboard at the venue once with internet
and capture over the areas you will show.

## How good is it

Measured on DOTA validation images resized to the camera's 0.3 m per pixel:
70-80 % of vehicles found in two of three car-park scenes (few false alarms),
about 20-25 % in the third, densely packed one; cars in residential driveways,
under trees and in shadow, are mostly missed. At 0.5 m per pixel the small
model still finds 60 %, at 0.7 m about a quarter. Planes, ships, storage
tanks, sports fields and pools are recognised too.
Time per picture (2048 x 1536 = 6 windows of 1024 px): about 4 s on a
2-core cloud machine, faster on a normal laptop; --model n halves it.

## Licence

See `models/LICENSE_NOTE.txt` - pretrained YOLO weights are AGPL-3.0 and DOTA
is academic: fine for a hackathon with published code, replace before a product.

## Recon-demo storyboard (real satellite pictures of the stops)

`make_storyboard.bat` (double-click) or `..\.venv\Scripts\python make_storyboard.py` takes the
balloon-camera picture of every stop in `demo_route.json` from Esri World Imagery, runs the detector
and opens `recon_storyboard.html`: the flight on a map of Bulgaria, the camera picture of each stop
with the detections, and the intel report. It needs internet for the imagery and takes a few minutes.

- It also fills `tile_cache` with the imagery of every stop: run it once with internet before the
  demo and the dashboard's recon pictures then work offline.
- The planned flight is Sofia -> Ruse on the dashboard's example wind field. Stops: Sofia Airport
  (while climbing), Hemus motorway at Vitinya, Lovech and the Osam bridges, Levski railway junction,
  and Ruse Airport (target, 20 km).
- The "operator" looks around each planned point (`search`, in picture widths) and centres the frame
  on the busiest spot. The page says how far it moved.
- To move a stop: right-click the spot in Google Maps, copy the coordinates into the stop's `lat`
  and `lon` in `demo_route.json`, then run `make_storyboard.py --only <stop number>`.
- `--model n` is about twice as fast.

