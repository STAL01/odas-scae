#!/usr/bin/env python3
"""
HAB-RECON recon-demo storyboard with REAL satellite pictures
=============================================================
Reads the planned flight and the recon stops from demo_route.json
(Sofia -> Ruse by default), takes the simulated balloon-camera picture of
every stop from Esri World Imagery, runs the same detector as the vision
server, and writes recon_storyboard.html - one self-contained page you can
open, send to the team or attach in the chat.

    ..\\.venv\\Scripts\\python make_storyboard.py          # all stops
    ..\\.venv\\Scripts\\python make_storyboard.py --only 2 # redo stop 2 after moving it
    ..\\.venv\\Scripts\\python make_storyboard.py --model n  # about 2x faster

To move a stop, change its "lat" and "lon" in demo_route.json (right-click the
spot in Google Maps to copy them) and run again with --only <number>.

The camera "operator" looks around the planned point (search = how many
picture widths) and centres the frame where the detector finds the most,
the way a gimbal operator would. The page says how far it moved.
"""
import argparse
import base64
import io
import json
import math
import os
import pathlib
import sys
import time
import webbrowser

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vision_server as vs  # noqa: E402  (same camera model and detector as the dashboard)

R_E = 6371000.0


def dist_brg(a, b):
    dn = math.radians(b[0] - a[0]) * R_E
    de = math.radians(b[1] - a[1]) * R_E * math.cos(math.radians((a[0] + b[0]) / 2))
    return math.hypot(dn, de), math.degrees(math.atan2(de, dn)) % 360


def balloon_at(route, t):
    pts = route['pts']
    for p in pts:
        if p[3] >= t:
            return p
    return pts[-1]


def geometry(route, s):
    """Where the balloon is at the stop time, and how far the camera looks."""
    lat, lon, z, _, _ = balloon_at(route, s['t'])
    d, brg = dist_brg([lat, lon], [s['lat'], s['lon']])
    h = max(100.0, z - s.get('ground', 0))
    slant = math.hypot(d, h)
    cam = route['camera']
    s.update(bal=[lat, lon, z], off_m=round(d), look_brg=round(brg), slant=round(slant),
             gsd=round(slant * cam['pixel_um'] * 1e-6 / (cam['focal_mm'] * 1e-3), 4),
             offnadir=round(math.degrees(math.atan2(d, h))))
    return s


def shoot(route, s, search, conf):
    """Same capture as the dashboard's recon stops: look around the stop, frame the busiest spot."""
    cam = route['camera']
    W, H = cam['sensor']
    search = vs.fit_search(s['lat'], s['gsd'], search, W, H)
    Wb, Hb = int(W * search) // 64 * 64, int(H * search) // 64 * 64
    big, meta = vs.camera_view(s['lat'], s['lon'], s['slant'], 0.0, cam['focal_mm'], realism=True, size=(Wb, Hb))
    dets, t_det, nwin = vs.detect(big, conf)
    x0, y0 = vs.best_frame(dets, W, H, Wb, Hb)
    frame = big.crop((x0, y0, x0 + W, y0 + H))
    gsd = meta['gsd']
    clat, clon = vs.pixel_to_ll(x0 + W / 2, y0 + H / 2, Wb, Hb, gsd, s['lat'], s['lon'], 0.0)
    k = 1280.0 / W
    out = []
    for cx, cy, w, h, sc, cl, ang in dets:
        if not (x0 + 4 <= cx < x0 + W - 4 and y0 + 4 <= cy < y0 + H - 4):
            continue
        la, lo = vs.pixel_to_ll(cx, cy, Wb, Hb, gsd, s['lat'], s['lon'], 0.0)
        out.append([round((cx - x0) * k, 1), round((cy - y0) * k, 1), round(w * k, 1), round(h * k, 1),
                    round(sc, 3), int(cl), round(ang, 4), round(la, 6), round(lo, 6)])
    disp = frame.resize((1280, round(1280 * H / W)), Image.LANCZOS)
    buf = io.BytesIO()
    disp.save(buf, 'JPEG', quality=82)
    return dict(img='data:image/jpeg;base64,' + base64.b64encode(buf.getvalue()).decode(),
                dets=out, center=[round(clat, 6), round(clon, 6)], shift_m=round(vs.dist_m([s['lat'], s['lon']], [clat, clon])),
                gsd=round(gsd, 4), source_mpp=round(meta['source_mpp'], 3), zoom=meta['zoom'],
                searched_m=[round(Wb * gsd), round(Hb * gsd)], search=search, windows=nwin, t_detect=round(t_det, 1),
                found_in_search=len(dets),
                source='mock image' if vs.ARGS.mock_image else 'Esri World Imagery (Esri, Maxar, Earthstar Geographics)')


def write_page(route, scenes, out, model):
    tpl = open(os.path.join(HERE, 'storyboard_template.html'), encoding='utf-8').read()
    meta = dict(made=time.strftime('%Y-%m-%d %H:%M'), model=f'yolo26{model}-obb')
    body = (tpl.replace('__ROUTE__', json.dumps(route, ensure_ascii=False, separators=(',', ':')))
               .replace('__SCENES__', json.dumps(scenes, separators=(',', ':')))
               .replace('__META__', json.dumps(meta)))
    html = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">\n'
            '</head>\n<body>\n' + body + '\n</body>\n</html>\n')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(html)
    return len(html)


def main():
    p = argparse.ArgumentParser(description='Recon-demo storyboard with real satellite pictures of each stop')
    p.add_argument('--route', default=os.path.join(HERE, 'demo_route.json'))
    p.add_argument('--out', default=os.path.join(HERE, 'recon_storyboard.html'))
    p.add_argument('--only', type=int, nargs='+', metavar='N', help='redo only these stops (keeps the others from the last run)')
    p.add_argument('--search', type=float, help='search area for every stop, in picture widths (default: per stop, 2.5-3)')
    p.add_argument('--model', choices=['n', 's'], default='s', help='s = better on small cars (default), n = about 2x faster')
    p.add_argument('--conf', type=float, default=0.30, help='detection confidence threshold')
    p.add_argument('--max-zoom', type=int, default=19, help='finest satellite zoom level to ask for')
    p.add_argument('--cache', default=os.path.join(HERE, 'tile_cache'))
    p.add_argument('--mock-image', help='offline test: use this north-up image instead of Esri')
    p.add_argument('--mock-gsd', type=float, default=0.3)
    p.add_argument('--mock-center', type=float, nargs=2, metavar=('LAT', 'LON'))
    p.add_argument('--no-browser', action='store_true')
    a = p.parse_args()

    route = json.load(open(a.route, encoding='utf-8'))
    cam = route['camera']
    vs.ARGS = argparse.Namespace(sensor=list(cam['sensor']), pixel_um=cam['pixel_um'], focal=cam['focal_mm'],
                                 max_zoom=a.max_zoom, cache=a.cache, mock_image=a.mock_image, mock_gsd=a.mock_gsd,
                                 mock_center=a.mock_center, model=a.model, conf=a.conf)
    vs.load_model()

    keep_path = os.path.join(a.cache, 'storyboard_scenes.json')
    scenes = {}
    if a.only and os.path.exists(keep_path):
        scenes = json.load(open(keep_path, encoding='utf-8'))
    for s in route['stops']:
        geometry(route, s)
        if a.only and s['id'] not in a.only and str(s['id']) in scenes:
            print(f"Stop {s['id']}: kept from the last run")
            continue
        search = vs.fit_search(s['lat'], s['gsd'], a.search or s.get('search', 2.5), *cam['sensor'])
        t0 = time.time()
        print(f"Stop {s['id']}: {s['name']}  ({s['lat']:.4f}, {s['lon']:.4f})  slant {s['slant'] / 1000:.1f} km, "
              f"{s['gsd']:.2f} m/px, looking around {search:g} picture widths ...", flush=True)
        try:
            sc = shoot(route, s, search, a.conf)
        except vs.NoImagery:
            print(f"   no satellite imagery here - move the stop a little and run again with --only {s['id']}")
            continue
        except OSError as e:
            print('   could not download the satellite tiles:', e)
            print('   check the internet connection (the imagery comes from server.arcgisonline.com)')
            sys.exit(1)
        scenes[str(s['id'])] = sc
        n = {}
        for d in sc['dets']:
            n[vs.CLASSES[d[5]]] = n.get(vs.CLASSES[d[5]], 0) + 1
        found = ', '.join(f'{v} {k}' for k, v in sorted(n.items(), key=lambda x: -x[1])) or 'nothing'
        print(f"   {found}  |  frame moved {sc['shift_m']} m from the planned point  |  satellite {sc['source_mpp']:.2f} m/px  "
              f"|  {time.time() - t0:.0f} s")
        if sum(1 for d in sc['dets'] if d[5] in (9, 10)) < 3 and s['kind'] != 'airport':
            print(f"   (few vehicles here - you can move this stop in demo_route.json and run again with --only {s['id']})")
    os.makedirs(a.cache, exist_ok=True)
    with open(keep_path, 'w', encoding='utf-8') as f:
        json.dump(scenes, f)
    size = write_page(route, scenes, a.out, a.model)
    print(f'\nWrote {a.out}  ({size / 1e6:.1f} MB)')
    if not a.no_browser:
        webbrowser.open(pathlib.Path(a.out).resolve().as_uri())   # handles spaces in the folder name


if __name__ == '__main__':
    main()
