#!/usr/bin/env python3
"""
HAB-RECON vision server - simulated balloon camera + vehicle detection
=====================================================================
The balloon is simulated, so its camera is too: this server builds the
picture the payload camera would take from real satellite imagery
(Esri World Imagery) at the balloon's position, altitude and heading,
adds what 20 km of air does to a picture (haze, blur, sensor noise),
and runs a vehicle detector on it - the same software that would run on
a real camera frame. Detections come back with latitude/longitude.

    pip install onnxruntime numpy pillow
    python vision_server.py                 # then open http://localhost:8765
    python vision_server.py --model n       # faster, finds fewer small cars
    python vision_server.py --mock-image my_photo.jpg --mock-gsd 0.3
                                            # offline: use your own satellite image
API: /api/capture?lat&lon&agl&heading&focal&conf&search   (search = look around N picture
     widths and centre on the busiest spot - the dashboard's recon stops use 2-3)

Camera model (change with --focal / --pixel-um / --sensor):
    250 mm lens, 3.45 um pixels, 2048 x 1536 sensor
    -> from 20 km: 0.27 m per pixel, picture 0.55 x 0.41 km
Detector: YOLO26-OBB pretrained on DOTA (15 classes incl. small/large vehicle).
    Weights: Ultralytics, AGPL-3.0; trained on DOTA (academic use).
    Fine for a hackathon demo with published code - see models/LICENSE_NOTE.txt.
"""
import argparse
import base64
import io
import json
import math
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
DASHBOARD_DIR = os.path.normpath(os.path.join(HERE, '..', 'dashboard'))
TILE_URL = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
CLASSES = ['plane', 'ship', 'storage tank', 'baseball diamond', 'tennis court', 'basketball court',
           'ground track field', 'harbor', 'bridge', 'large vehicle', 'small vehicle', 'helicopter',
           'roundabout', 'soccer ball field', 'swimming pool']
R_E = 6371000.0

ARGS = None
SESSION = None
DETECT_LOCK = threading.Lock()


class NoImagery(Exception):
    pass


# ----------------------------------------------------------------- tiles
def metres_per_px(lat, z):
    return 156543.03392 * math.cos(math.radians(lat)) / 2 ** z


def ll_to_px(lat, lon, z):
    n = 256 * 2 ** z
    s = math.sin(math.radians(lat))
    return (lon + 180.0) / 360.0 * n, (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n


def is_placeholder(img):
    # Esri answers "Map data not yet available" with a light grey tile carrying a little text:
    # nearly every pixel has the same grey level
    a = np.asarray(img.convert('L'), dtype=np.float32)
    med = float(np.median(a))
    return med > 150 and float(np.mean(np.abs(a - med) <= 3)) > 0.85


def get_tile(z, x, y):
    path = os.path.join(ARGS.cache, str(z), str(x), f'{y}.jpg')
    if os.path.exists(path):
        if os.path.getsize(path) == 0:
            raise NoImagery
        return Image.open(path).convert('RGB')
    req = urllib.request.Request(TILE_URL.format(z=z, x=x, y=y), headers={'User-Agent': 'HAB-RECON-hackathon-demo/1.0'})
    try:
        data = urllib.request.urlopen(req, timeout=12).read()
    except urllib.error.HTTPError as e:
        if e.code == 404:
            raise NoImagery
        raise
    img = Image.open(io.BytesIO(data)).convert('RGB')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if is_placeholder(img):
        open(path, 'wb').close()   # remember "no imagery here"
        raise NoImagery
    with open(path, 'wb') as f:
        f.write(data)
    return img


def mosaic_for(lat, lon, radius_m, z):
    """Satellite mosaic around (lat, lon) at zoom z; returns image, centre px, metres/px."""
    mpp = metres_per_px(lat, z)
    cx, cy = ll_to_px(lat, lon, z)
    r = radius_m / mpp + 2
    tx0, tx1 = int((cx - r) // 256), int((cx + r) // 256)
    ty0, ty1 = int((cy - r) // 256), int((cy + r) // 256)
    tiles = [(tx, ty) for ty in range(ty0, ty1 + 1) for tx in range(tx0, tx1 + 1)]
    if len(tiles) > 900:
        raise ValueError('picture too large for this zoom')
    mos = Image.new('RGB', ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
    missing = 0
    with ThreadPoolExecutor(max_workers=12) as ex:
        for (tx, ty), img in zip(tiles, ex.map(lambda t: _tile_or_none(z, *t), tiles)):
            if img is None:
                missing += 1
            else:
                mos.paste(img, ((tx - tx0) * 256, (ty - ty0) * 256))
    if missing > len(tiles) * 0.25:
        raise NoImagery
    return mos, (cx - tx0 * 256, cy - ty0 * 256), mpp, len(tiles)


def _tile_or_none(z, x, y):
    try:
        return get_tile(z, x, y)
    except NoImagery:
        return None


def mock_mosaic(lat, lon):
    img = Image.open(ARGS.mock_image).convert('RGB')
    c = ARGS.mock_center or (lat, lon)
    dn = (lat - c[0]) * math.pi / 180 * R_E
    de = (lon - c[1]) * math.pi / 180 * R_E * math.cos(math.radians(c[0]))
    return img, (img.width / 2 + de / ARGS.mock_gsd, img.height / 2 - dn / ARGS.mock_gsd), ARGS.mock_gsd, 0


# ----------------------------------------------------------------- camera
def camera_gsd(agl, focal_mm):
    return max(0.05, agl) * ARGS.pixel_um * 1e-6 / (focal_mm * 1e-3)


def camera_view(lat, lon, agl, heading, focal_mm, realism=True, size=None):
    """The picture a nadir camera at (lat, lon, agl) with the image top toward 'heading' takes.
    size: picture size in pixels (default: the sensor; bigger when the operator searches around)."""
    W, H = size or ARGS.sensor
    gsd = camera_gsd(agl, focal_mm)
    radius = 0.5 * math.hypot(W, H) * gsd
    t0 = time.time()
    if ARGS.mock_image:
        mos, (Cx, Cy), mpp, ntiles = mock_mosaic(lat, lon)
        zoom = None
    else:
        zwant = math.ceil(math.log2(156543.03392 * math.cos(math.radians(lat)) / gsd))
        zoom = max(3, min(ARGS.max_zoom, zwant))
        while True:
            try:
                mos, (Cx, Cy), mpp, ntiles = mosaic_for(lat, lon, radius, zoom)
                break
            except (NoImagery, ValueError):
                zoom -= 1
                if zoom < max(3, zwant - 4):
                    raise NoImagery
    t_fetch = time.time() - t0
    k = gsd / mpp                                   # mosaic pixels per camera pixel
    if k > 1.5:                                     # shrink first so the resampling doesn't alias
        f = int(k)
        mos = mos.reduce(f)
        Cx, Cy, k = Cx / f, Cy / f, k / f
    psi = math.radians(heading)
    a, b, d, e = k * math.cos(psi), -k * math.sin(psi), k * math.sin(psi), k * math.cos(psi)
    c, f_ = Cx - a * W / 2 - b * H / 2, Cy - d * W / 2 - e * H / 2
    img = mos.transform((W, H), Image.AFFINE, (a, b, c, d, e, f_), resample=Image.BICUBIC)
    if realism:
        # 20 km of air: some haze (lower contrast, bluish), a little blur, sensor noise
        arr = np.asarray(img.filter(ImageFilter.GaussianBlur(0.6)), dtype=np.float32)
        haze = min(0.18, 0.012 * agl / 1000)
        arr = arr * (1 - haze) + np.array([172, 186, 206], np.float32) * haze
        arr += np.random.default_rng().normal(0, 2.2, arr.shape)
        img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    return img, dict(gsd=gsd, source_mpp=mpp if zoom is None else metres_per_px(lat, zoom), zoom=zoom,
                     tiles=ntiles, t_fetch=t_fetch, footprint_m=[W * gsd, H * gsd])


def pixel_to_ll(u, v, W, H, gsd, lat, lon, heading):
    psi = math.radians(heading)
    fwd, right = (H / 2 - v) * gsd, (u - W / 2) * gsd
    n = fwd * math.cos(psi) - right * math.sin(psi)
    e = fwd * math.sin(psi) + right * math.cos(psi)
    return lat + math.degrees(n / R_E), lon + math.degrees(e / (R_E * math.cos(math.radians(lat))))


# ----------------------------------------------------------------- operator framing
# What the camera operator centres on when looking around a point: planes and
# ships matter more than one more car. Weights per DOTA class.
FRAME_WEIGHT = {0: 8, 11: 6, 1: 5, 8: 4, 7: 3, 2: 2, 12: 2, 9: 1.5, 10: 1}


def fit_search(lat, gsd, search, W, H, max_tiles=850):
    """Shrink the search area until the satellite mosaic at full resolution stays under the tile limit."""
    if ARGS.mock_image or search <= 1:
        return max(1.0, search)
    zwant = min(ARGS.max_zoom, math.ceil(math.log2(156543.03392 * math.cos(math.radians(lat)) / gsd)))
    mpp = metres_per_px(lat, zwant)
    while search > 1.0:
        side = math.hypot(W * search, H * search) * gsd / mpp      # what mosaic_for asks for
        if (side / 256 + 2) ** 2 <= max_tiles:
            break
        search -= 0.25
    return max(1.0, search)


def best_frame(dets, W, H, Wb, Hb, step=64):
    """Top-left corner of the W x H frame inside the Wb x Hb search picture with the most to see."""
    if not dets or (Wb <= W and Hb <= H):
        return (Wb - W) // 2, (Hb - H) // 2
    P = np.array([[d[0], d[1], FRAME_WEIGHT.get(d[5], 0.1)] for d in dets], dtype=np.float32)
    xs = np.arange(0, Wb - W + 1, step, dtype=np.float32)
    ys = np.arange(0, Hb - H + 1, step, dtype=np.float32)
    gx, gy = np.meshgrid(xs, ys)
    gx, gy = gx.ravel(), gy.ravel()
    m = 24  # an object counts when its centre is this far inside the frame
    inside = ((P[None, :, 0] >= gx[:, None] + m) & (P[None, :, 0] < gx[:, None] + W - m) &
              (P[None, :, 1] >= gy[:, None] + m) & (P[None, :, 1] < gy[:, None] + H - m))
    score = (inside * P[None, :, 2]).sum(1)
    # prefer the planned point when it is about as good
    r = np.hypot(gx + W / 2 - Wb / 2, gy + H / 2 - Hb / 2) / max(1.0, math.hypot(Wb - W, Hb - H) / 2)
    i = int(np.argmax(score * (1 - 0.25 * r) + 1e-3 * (1 - r)))
    return int(gx[i]), int(gy[i])


def dist_m(a, b):
    dn = math.radians(b[0] - a[0]) * R_E
    de = math.radians(b[1] - a[1]) * R_E * math.cos(math.radians((a[0] + b[0]) / 2))
    return math.hypot(dn, de)


# ----------------------------------------------------------------- detector
def load_model():
    global SESSION
    import onnxruntime as ort
    path = os.path.join(HERE, 'models', f'yolo26{ARGS.model}-obb.onnx')
    if not os.path.exists(path):
        raise SystemExit(f'Model not found: {path}')
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    avail = ort.get_available_providers()
    prefer = [p for p in ('CUDAExecutionProvider', 'DmlExecutionProvider', 'CoreMLExecutionProvider') if p in avail]
    SESSION = ort.InferenceSession(path, so, providers=prefer + ['CPUExecutionProvider'])
    print(f'Detector: {os.path.basename(path)} on {SESSION.get_providers()[0]}')


def detect(img, conf):
    """Slide 1024x1024 windows over the picture (the model's size), merge the results."""
    W, H = img.size
    T = 1024
    xs = [0] if W <= T else sorted(set(list(range(0, W - T, T // 2)) + [W - T]))
    ys = [0] if H <= T else sorted(set(list(range(0, H - T, T // 2)) + [H - T]))
    dets = []
    t0 = time.time()
    for y0 in ys:
        for x0 in xs:
            crop = img.crop((x0, y0, x0 + T, y0 + T))          # pads with black past the edge
            x = np.asarray(crop, dtype=np.float32).transpose(2, 0, 1)[None] / 255.0
            with DETECT_LOCK:
                out = SESSION.run(None, {SESSION.get_inputs()[0].name: x})[0][0]
            for cx, cy, w, h, s, cl, ang in out[out[:, 4] >= conf]:
                # skip objects cut by an inner window edge: another window sees them whole
                edge = 6
                if (cx - w / 2 < edge and x0 > 0) or (cy - h / 2 < edge and y0 > 0) or \
                   (cx + w / 2 > T - edge and x0 + T < W) or (cy + h / 2 > T - edge and y0 + T < H):
                    continue
                if cx + x0 >= W or cy + y0 >= H:
                    continue
                dets.append([float(cx + x0), float(cy + y0), float(w), float(h), float(s), int(cl), float(ang)])
    dets.sort(key=lambda d: -d[4])
    keep = []
    for d in dets:                                             # same object seen by two windows
        if all((d[0] - k[0]) ** 2 + (d[1] - k[1]) ** 2 > (0.45 * max(k[2], k[3], d[2], d[3])) ** 2 for k in keep):
            keep.append(d)
    return keep, time.time() - t0, len(xs) * len(ys)


def jpeg_data_url(img, q=85):
    buf = io.BytesIO()
    img.save(buf, 'JPEG', quality=q)
    return 'data:image/jpeg;base64,' + base64.b64encode(buf.getvalue()).decode()


def capture(q):
    """Camera picture at (lat, lon) from 'agl' metres (slant range for a gimbal shot) plus detections.
    search > 1: the operator looks around search x the picture size and centres the frame where the
    detector finds the most (planes and ships count more than cars)."""
    lat, lon = float(q['lat']), float(q['lon'])
    agl = float(q.get('agl', 19400))
    heading = float(q.get('heading', 0)) % 360
    focal = float(q.get('focal', ARGS.focal))
    conf = float(q.get('conf', ARGS.conf))
    W, H = ARGS.sensor
    search = fit_search(lat, camera_gsd(agl, focal), max(1.0, min(4.0, float(q.get('search', 1)))), W, H)
    Wb, Hb = (int(W * search) // 64 * 64, int(H * search) // 64 * 64) if search > 1 else (W, H)
    big, meta = camera_view(lat, lon, agl, heading, focal, q.get('realism', '1') != '0', size=(Wb, Hb))
    dets, t_det, nwin = detect(big, conf)
    x0, y0 = best_frame(dets, W, H, Wb, Hb)
    img = big.crop((x0, y0, x0 + W, y0 + H)) if (Wb, Hb) != (W, H) else big
    gsd = meta['gsd']
    ll = lambda u, v: pixel_to_ll(x0 + u, y0 + v, Wb, Hb, gsd, lat, lon, heading)
    out = []
    for cx, cy, w, h, s, cl, ang in dets:
        u, v = cx - x0, cy - y0
        if not (0 <= u < W and 0 <= v < H):
            continue
        la, lo = ll(u, v)
        out.append(dict(cls=cl, name=CLASSES[cl], conf=round(s, 3), u=round(u, 1), v=round(v, 1), w=round(w, 1), h=round(h, 1),
                        angle=round(math.degrees(ang), 1), lat=round(la, 7), lon=round(lo, 7),
                        length_m=round(max(w, h) * gsd, 1), width_m=round(min(w, h) * gsd, 1)))
    corners = [ll(u, v) for u, v in ((0, 0), (W, 0), (W, H), (0, H))]
    clat, clon = ll(W / 2, H / 2)
    return dict(ok=True, image=jpeg_data_url(img), w=W, h=H, lat=clat, lon=clon, aim=[lat, lon], agl=agl, heading=heading, focal=focal,
                gsd=gsd, source_mpp=meta['source_mpp'], zoom=meta['zoom'], tiles=meta['tiles'],
                footprint=corners, footprint_m=[W * gsd, H * gsd], t_fetch=round(meta['t_fetch'], 2),
                search=search, searched_m=[round(Wb * gsd), round(Hb * gsd)], shift_m=round(dist_m([lat, lon], [clat, clon])),
                found_in_search=len(dets), t_detect=round(t_det, 2), windows=nwin, model=f'yolo26{ARGS.model}-obb', detections=out,
                source='mock image' if ARGS.mock_image else 'Esri World Imagery (Esri, Maxar, Earthstar Geographics)')


def detect_uploaded(body, q):
    img = Image.open(io.BytesIO(body)).convert('RGB')
    if max(img.size) > 6000:
        img.thumbnail((6000, 6000))
    gsd = float(q['gsd']) if q.get('gsd') else None
    dets, t_det, nwin = detect(img, float(q.get('conf', ARGS.conf)))
    out = [dict(cls=cl, name=CLASSES[cl], conf=round(s, 3), u=round(cx, 1), v=round(cy, 1), w=round(w, 1), h=round(h, 1),
                angle=round(math.degrees(a), 1), length_m=round(max(w, h) * gsd, 1) if gsd else None)
           for cx, cy, w, h, s, cl, a in dets]
    return dict(ok=True, image=jpeg_data_url(img), w=img.width, h=img.height, gsd=gsd, t_detect=round(t_det, 2),
                windows=nwin, model=f'yolo26{ARGS.model}-obb', detections=out, source='uploaded image')


# ----------------------------------------------------------------- http
class Handler(BaseHTTPRequestHandler):
    server_version = 'HAB-RECON-vision/1.0'

    def log_message(self, fmt, *a):
        if '/api/health' not in (a[0] if a else ''):
            print(time.strftime('%H:%M:%S'), fmt % a)

    def _headers(self, code, ctype, length):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(length))
        self.send_header('Access-Control-Allow-Origin', '*')       # the dashboard may be opened as a file
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()

    def _json(self, obj, code=200):
        data = json.dumps(obj).encode()
        self._headers(code, 'application/json', len(data))
        self.wfile.write(data)

    def do_OPTIONS(self):
        self._headers(204, 'text/plain', 0)

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        q = dict(urllib.parse.parse_qsl(url.query))
        try:
            if url.path == '/api/health':
                return self._json(dict(ok=True, model=f'yolo26{ARGS.model}-obb', provider=SESSION.get_providers()[0],
                                       source='mock image' if ARGS.mock_image else 'Esri World Imagery',
                                       focal=ARGS.focal, pixel_um=ARGS.pixel_um, sensor=ARGS.sensor, conf=ARGS.conf))
            if url.path == '/api/capture':
                return self._json(capture(q))
            return self._static(url.path)
        except NoImagery:
            return self._json(dict(ok=False, error='No satellite imagery for this spot at this resolution'), 404)
        except Exception as e:  # report, keep serving
            return self._json(dict(ok=False, error=f'{type(e).__name__}: {e}'), 500)

    def do_POST(self):
        url = urllib.parse.urlparse(self.path)
        q = dict(urllib.parse.parse_qsl(url.query))
        try:
            if url.path == '/api/detect':
                n = int(self.headers.get('Content-Length', 0))
                if n <= 0 or n > 60_000_000:
                    return self._json(dict(ok=False, error='send one image, max 60 MB'), 400)
                return self._json(detect_uploaded(self.rfile.read(n), q))
            return self._json(dict(ok=False, error='unknown endpoint'), 404)
        except Exception as e:
            return self._json(dict(ok=False, error=f'{type(e).__name__}: {e}'), 500)

    def _static(self, path):
        if path in ('', '/'):
            path = '/dashboard.html'
        full = os.path.normpath(os.path.join(DASHBOARD_DIR, urllib.parse.unquote(path).lstrip('/')))
        if not full.startswith(DASHBOARD_DIR + os.sep) or not os.path.isfile(full):
            return self._json(dict(ok=False, error='not found'), 404)
        ctype = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.css': 'text/css',
                 '.png': 'image/png', '.jpg': 'image/jpeg', '.json': 'application/json'}.get(os.path.splitext(full)[1], 'application/octet-stream')
        with open(full, 'rb') as f:
            data = f.read()
        self._headers(200, ctype, len(data))
        self.wfile.write(data)


def self_test():
    if ARGS.mock_image:
        print(f'Imagery: mock image {ARGS.mock_image} at {ARGS.mock_gsd} m/px')
        return
    try:
        x, y = ll_to_px(42.6977, 23.3219, 15)   # a tile over Sofia
        get_tile(15, int(x // 256), int(y // 256))
        print('Imagery: Esri World Imagery reachable')
    except Exception as e:
        print(f'Imagery: WARNING - cannot reach Esri World Imagery ({e}).\n'
              f'         Already cached areas still work. Offline: --mock-image your_image.jpg --mock-gsd 0.3')


def main():
    global ARGS
    p = argparse.ArgumentParser(description='HAB-RECON simulated camera + vehicle detection')
    p.add_argument('--port', type=int, default=8765)
    p.add_argument('--model', choices=['n', 's'], default='s', help='s = better on small cars (default), n = ~2x faster')
    p.add_argument('--conf', type=float, default=0.30, help='detection confidence threshold')
    p.add_argument('--focal', type=float, default=250.0, help='lens focal length, mm')
    p.add_argument('--pixel-um', type=float, default=3.45, help='sensor pixel size, micrometres')
    p.add_argument('--sensor', type=int, nargs=2, default=[2048, 1536], metavar=('W', 'H'))
    p.add_argument('--max-zoom', type=int, default=19, help='finest satellite zoom level to ask for')
    p.add_argument('--cache', default=os.path.join(HERE, 'tile_cache'))
    p.add_argument('--mock-image', help='use this north-up satellite image instead of Esri (offline)')
    p.add_argument('--mock-gsd', type=float, default=0.3, help='metres per pixel of --mock-image')
    p.add_argument('--mock-center', type=float, nargs=2, metavar=('LAT', 'LON'), help='where the mock image is (default: under the camera)')
    p.add_argument('--no-browser', action='store_true')
    ARGS = p.parse_args()
    load_model()
    self_test()
    srv = ThreadingHTTPServer(('127.0.0.1', ARGS.port), Handler)
    url = f'http://localhost:{ARGS.port}/'
    print(f'Vision server running - open {url}  (Ctrl+C to stop)')
    if not ARGS.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('stopped')


if __name__ == '__main__':
    main()
