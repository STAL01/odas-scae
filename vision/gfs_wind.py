#!/usr/bin/env python3
"""
HAB-RECON wind data - NOAA GFS 0.25 deg forecast for the balloon simulation
===========================================================================
Downloads the NOAA Global Forecast System (GFS) forecast for a box around the
launch site from NOMADS, decodes the GRIB2 files (pure numpy, no extra
packages), and stores one compact file per forecast in vision/wind_cache.
The vision server hands that file to the dashboard (/api/wind), where the
mission plan, demo flight and swarm simulation fly through it.

Per forecast step: wind (u, v), geopotential height and temperature on 28
pressure levels, 1000 hPa (near the ground) to 10 hPa (about 31 km), plus the
terrain height. Default box: about +-5 deg latitude and +-7 deg longitude
around the launch site, 3-hourly steps for 48 h (17 files, ~8 MB download).

    python gfs_wind.py --lat 42.66 --lon 23.36                  # next 48 h
    python gfs_wind.py --lat 42.66 --lon 23.36 --hours 120      # 5 days, for an offline demo
    python gfs_wind.py --lat 42.66 --lon 23.36 --start "2026-10-21 08:00"   # UTC
    python gfs_wind.py --decode some_file.grib2                 # list the fields in a GRIB2 file

Data: NOAA/NCEP GFS, public domain (attribution requested, no endorsement implied).
NOMADS asks for fewer than ~120 requests a minute; this script makes about one a second.
"""
import argparse
import datetime as dt
import glob
import io
import json
import math
import os
import struct
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

import numpy as np

NOMADS = os.environ.get('HAB_NOMADS', 'https://nomads.ncep.noaa.gov')
LEVELS_HPA = [1000, 975, 950, 925, 900, 850, 800, 750, 700, 650, 600, 550, 500, 450, 400, 350,
              300, 250, 200, 150, 100, 70, 50, 40, 30, 20, 15, 10]
VARS = ['gh', 'u', 'v', 't']                 # stored per level in this order
SCALE = [1.0, 0.01, 0.01, 0.01]               # value = raw * scale + offset   (raw = int16)
OFFSET = [16000.0, 0.0, 0.0, 250.0]           # gh in m, u/v in m/s, t in K
PARAMS = {(0, 3, 5): 'gh', (0, 2, 2): 'u', (0, 2, 3): 'v', (0, 0, 0): 't'}   # (discipline, category, number)
NOMADS_VAR = {'gh': 'HGT', 'u': 'UGRD', 'v': 'VGRD', 't': 'TMP'}
UA = 'HAB-RECON-hackathon/1.0 (balloon trajectory simulation)'
MAGIC = b'HABW'


# =====================================================================
#  GRIB2 decoder (WMO FM 92 GRIB edition 2) - the subset NCEP uses:
#  grid 3.0 (regular lat/lon), product 4.0, data templates 5.0 (simple),
#  5.2/5.3 (complex packing, spatial differencing), 5.40 (JPEG 2000) and
#  5.41 (PNG) - the last two through Pillow.
# =====================================================================
class GribError(Exception):
    pass


def _u(b, o, n):
    return int.from_bytes(b[o:o + n], 'big')


def _s(b, o, n):
    """GRIB2 signed integer: sign bit + magnitude (not two's complement)."""
    v = int.from_bytes(b[o:o + n], 'big')
    m = 1 << (8 * n - 1)
    return -(v & (m - 1)) if v & m else v


def _bits(buf, pos, width):
    """Unsigned integers of 'width' bits (array or scalar) at bit positions 'pos' (array).
    buf: uint8 array padded with >= 8 zero bytes."""
    pos = np.asarray(pos, dtype=np.int64)
    width = np.broadcast_to(np.asarray(width, dtype=np.int64), pos.shape)
    if pos.size == 0:
        return np.zeros(0, dtype=np.int64)
    if int(width.max()) > 56:
        raise GribError('bit field wider than 56 bits')
    byte = pos >> 3
    word = np.zeros(pos.shape, dtype=np.uint64)
    for k in range(8):
        word |= buf[byte + k].astype(np.uint64) << np.uint64(56 - 8 * k)
    w = width.astype(np.uint64)
    shift = np.uint64(64) - (pos & 7).astype(np.uint64) - w
    shift = np.where(width == 0, np.uint64(0), shift)
    mask = np.where(width == 0, np.uint64(0), (np.uint64(1) << w) - np.uint64(1))
    return ((word >> shift) & mask).astype(np.int64)


def _ceil8(p):
    return (p + 7) // 8 * 8


def _unpack_simple(s5, data, n):
    nbits = s5[19]
    if nbits == 0:
        return np.zeros(n, dtype=np.int64), None
    buf = np.frombuffer(bytes(data) + b'\0' * 8, dtype=np.uint8)
    return _bits(buf, np.arange(n, dtype=np.int64) * nbits, nbits), None


def _unpack_complex(s5, data, n, tmpl):
    nbits, miss = s5[19], s5[22]
    ng = _u(s5, 31, 4)
    wref, wbits = s5[35], s5[36]
    lref, linc, llast, lbits = _u(s5, 37, 4), s5[41], _u(s5, 42, 4), s5[46]
    order = noct = 0
    pos = 0
    ivals, minsd = [], 0
    if tmpl == 3:
        order, noct = s5[47], s5[48]
        for _ in range(order):
            ivals.append(_s(data, pos // 8, noct))
            pos += noct * 8
        minsd = _s(data, pos // 8, noct) if noct else 0
        pos += noct * 8
    buf = np.frombuffer(bytes(data) + b'\0' * 8, dtype=np.uint8)
    ar = np.arange(ng, dtype=np.int64)
    gref = _bits(buf, pos + ar * nbits, nbits) if nbits else np.zeros(ng, dtype=np.int64)
    pos = _ceil8(pos + ng * nbits)
    gw = wref + (_bits(buf, pos + ar * wbits, wbits) if wbits else np.zeros(ng, dtype=np.int64))
    pos = _ceil8(pos + ng * wbits)
    gl = lref + linc * (_bits(buf, pos + ar * lbits, lbits) if lbits else np.zeros(ng, dtype=np.int64))
    pos = _ceil8(pos + ng * lbits)
    if ng:
        gl[-1] = llast
    if int(gl.sum()) != n:
        raise GribError(f'complex packing: groups hold {int(gl.sum())} values, expected {n}')
    widths = np.repeat(gw, gl)
    starts = pos + np.concatenate(([0], np.cumsum(widths)[:-1])) if n else np.zeros(0, dtype=np.int64)
    x2 = _bits(buf, starts, widths)
    gref_v, gw_v = np.repeat(gref, gl), widths
    x = gref_v + x2
    missing = None
    if miss in (1, 2):
        allones_ref = (1 << nbits) - 1 if nbits else 0
        m1 = np.where(gw_v == 0, gref_v == allones_ref, x2 == (np.int64(1) << gw_v) - 1)
        missing = m1
        if miss == 2:
            m2 = np.where(gw_v == 0, gref_v == allones_ref - 1, x2 == (np.int64(1) << gw_v) - 2)
            missing = m1 | m2
    if tmpl == 3 and order:
        good = np.flatnonzero(~missing) if missing is not None else np.arange(n)
        v = x[good].astype(np.int64)
        if v.size:
            if order == 1:
                v[0] = ivals[0]
                v[1:] += minsd
                v = np.cumsum(v)
            elif order == 2:
                v[0] = ivals[0]
                if v.size > 1:
                    v[1] = ivals[1]
                    v[2:] += minsd
                    d = v.copy()
                    d[1] = v[1] - v[0]
                    d[1:] = np.cumsum(d[1:])          # first differences
                    v = np.cumsum(d)                 # values
            else:
                raise GribError(f'spatial differencing order {order} not supported')
        x = np.zeros(n, dtype=np.int64)
        x[good] = v
    return x, missing


def _unpack_image(s5, data, n, tmpl):
    from PIL import Image                     # Pillow wheels include JPEG 2000 (OpenJPEG) and PNG
    nbits = s5[19]
    if nbits == 0:
        return np.zeros(n, dtype=np.int64), None
    img = Image.open(io.BytesIO(bytes(data)))
    a = np.asarray(img)
    if tmpl == 40 and a.ndim == 2:            # Pillow stretches odd precisions to fill 8 or 16 bits
        container = 8 if img.mode == 'L' else 16 if img.mode.startswith('I;16') else 0
        if container and nbits < container:
            a = a.astype(np.int64) >> (container - nbits)
    if a.ndim == 3:                           # 24/32-bit PNG: channels hold the bytes of each value
        a = a.astype(np.int64)
        v = np.zeros(a.shape[:2], dtype=np.int64)
        for c in range(a.shape[2]):
            v = (v << 8) | a[..., c]
        a = v
    a = a.astype(np.int64).ravel()
    if a.size != n:
        raise GribError(f'image packing: {a.size} values, expected {n}')
    return a, None


def grib2_messages(blob):
    """Yield one dict per GRIB2 message: param key, level, times, grid and values (2-D, south->north rows)."""
    b = memoryview(blob)
    off, bitmap_prev = 0, None
    while True:
        off = blob.find(b'GRIB', off)
        if off < 0:
            return
        if b[off + 7] != 2:
            raise GribError(f'GRIB edition {b[off + 7]} - only edition 2 is supported')
        total = _u(b, off + 8, 8)
        msg = b[off:off + total]
        if len(msg) < total:
            raise GribError('truncated GRIB message')
        disc = msg[6]
        p = 16
        info = dict(discipline=disc)
        grid = None
        while p < total - 4:
            ln, num = _u(msg, p, 4), msg[p + 4]
            s = msg[p:p + ln]
            if num == 1:
                info['ref'] = dt.datetime(_u(s, 12, 2), s[14], s[15], s[16], s[17], s[18], tzinfo=dt.timezone.utc)
            elif num == 3:
                tpl = _u(s, 12, 2)
                if tpl != 0:
                    raise GribError(f'grid template 3.{tpl} not supported (only regular lat/lon)')
                npts = _u(s, 6, 4)
                ni, nj = _u(s, 30, 4), _u(s, 34, 4)
                basic, sub = _u(s, 38, 4), _u(s, 42, 4)
                unit = 1e-6 if basic in (0, 0xFFFFFFFF) or sub in (0, 0xFFFFFFFF) else basic / sub
                grid = dict(npts=npts, ni=ni, nj=nj, la1=_s(s, 46, 4) * unit, lo1=_s(s, 50, 4) * unit,
                            la2=_s(s, 55, 4) * unit, lo2=_s(s, 59, 4) * unit,
                            di=_u(s, 63, 4) * unit, dj=_u(s, 67, 4) * unit, scan=s[71])
            elif num == 4:
                tpl = _u(s, 7, 2)
                if tpl not in (0, 1, 8, 11):
                    raise GribError(f'product template 4.{tpl} not supported')
                unit = {0: 60, 1: 3600, 2: 86400, 10: 3 * 3600, 11: 6 * 3600, 12: 12 * 3600, 13: 1}.get(s[17], 3600)
                sf, sv = _s(s, 23, 1), _u(s, 24, 4)
                info.update(cat=s[9], num=s[10], fsec=_u(s, 18, 4) * unit, ltype=s[22],
                            lval=sv / 10 ** sf if sv != 0xFFFFFFFF else None)
            elif num == 5:
                info['s5'] = bytes(s)
                info['ndata'] = _u(s, 5, 4)
                info['dtpl'] = _u(s, 9, 2)
            elif num == 6:
                ind = s[5]
                if ind == 0:
                    bm = np.unpackbits(np.frombuffer(bytes(s[6:]), dtype=np.uint8))[:grid['npts']].astype(bool)
                    info['bitmap'] = bitmap_prev = bm
                elif ind == 254:
                    info['bitmap'] = bitmap_prev
                else:
                    info['bitmap'] = None
            elif num == 7:
                s5, n, tpl = info['s5'], info['ndata'], info['dtpl']
                data = s[5:]
                if tpl == 0:
                    x, miss = _unpack_simple(s5, data, n)
                elif tpl in (2, 3):
                    x, miss = _unpack_complex(s5, data, n, tpl)
                elif tpl in (40, 41):
                    x, miss = _unpack_image(s5, data, n, tpl)
                else:
                    raise GribError(f'data template 5.{tpl} not supported')
                R = struct.unpack('>f', s5[11:15])[0]
                E, D = _s(s5, 15, 2), _s(s5, 17, 2)
                vals = (R + x.astype(np.float64) * 2.0 ** E) / 10.0 ** D
                if miss is not None:
                    vals[miss] = np.nan
                bm = info.get('bitmap')
                if bm is not None:
                    full = np.full(grid['npts'], np.nan)
                    full[bm] = vals
                    vals = full
                info['values'] = vals
            p += ln
        if grid is None or 'values' not in info:
            raise GribError('GRIB message without grid or data')
        yield _shape(info, grid)
        off += total


def _shape(info, g):
    scan = g['scan']
    if scan & 0x20 or scan & 0x10:
        raise GribError(f'scanning mode {scan:#x} not supported')
    a = info['values'].reshape(g['nj'], g['ni'])
    if not scan & 0x40:          # rows run north -> south: turn them over
        a = a[::-1]
    if scan & 0x80:              # columns run east -> west
        a = a[:, ::-1]
    lat0 = min(g['la1'], g['la2'])
    lon0 = g['lo1'] if not scan & 0x80 else g['lo2']
    key = PARAMS.get((info['discipline'], info.get('cat'), info.get('num')))
    lt, lv = info.get('ltype'), info.get('lval')
    level = (('hpa', round(lv / 100, 3)) if lt == 100 else ('surface', 0) if lt == 1
             else ('agl', lv) if lt == 103 else (str(lt), lv))
    return dict(key=key, param=(info['discipline'], info.get('cat'), info.get('num')), level=level,
                ref=info.get('ref'), valid=info['ref'] + dt.timedelta(seconds=info.get('fsec', 0)),
                lat0=lat0, lon0=lon0 % 360, dlat=g['dj'], dlon=g['di'], ny=g['nj'], nx=g['ni'],
                values=a.astype(np.float32))


# =====================================================================
#  NOMADS download
# =====================================================================
def _open(url, method='GET', timeout=30):
    req = urllib.request.Request(url, method=method, headers={'User-Agent': UA})
    return urllib.request.urlopen(req, timeout=timeout)


def file_exists(run, fh):
    d, hh = run.strftime('%Y%m%d'), run.strftime('%H')
    url = f'{NOMADS}/pub/data/nccf/com/gfs/prod/gfs.{d}/{hh}/atmos/gfs.t{hh}z.pgrb2.0p25.f{fh:03d}.idx'
    try:
        with _open(url, 'HEAD', timeout=15) as r:
            return r.status == 200
    except urllib.error.HTTPError as e:
        if e.code == 405:          # HEAD not allowed: fetch the small index file instead
            try:
                with _open(url, timeout=20) as r:
                    return r.status == 200
            except urllib.error.HTTPError:
                return False
        if e.code in (403, 429):
            raise ConnectionError(f'NOMADS refused the request (HTTP {e.code}) - too many requests? Wait 10 minutes.')
        return False


def steps_for(run, start, hours, step):
    lead0 = (start - run).total_seconds() / 3600
    f0 = max(0, int(math.floor(lead0 / step)) * step)
    f1 = int(math.ceil((lead0 + hours) / step)) * step
    return list(range(f0, f1 + 1, step))


def find_run(start, hours, step, log=print):
    """Newest GFS run at or before 'start' whose files reach start + hours."""
    now = dt.datetime.now(dt.timezone.utc)
    t = min(start, now)
    run = t.replace(hour=t.hour // 6 * 6, minute=0, second=0, microsecond=0)
    for _ in range(40):                       # NOMADS keeps about 10 days
        fh = steps_for(run, start, hours, step)
        if fh[-1] <= 384 and (step == 3 or fh[-1] <= 120):
            if file_exists(run, fh[-1]) and file_exists(run, fh[0]):
                return run, fh
        log(f'  GFS {run:%d %b %H}Z not available (yet) for this time - trying the run before')
        run -= dt.timedelta(hours=6)
    raise LookupError('no GFS run on NOMADS covers this time (NOMADS keeps about 10 days)')


def filter_url(run, fh, box):
    d, hh = run.strftime('%Y%m%d'), run.strftime('%H')
    q = [('dir', f'/gfs.{d}/{hh}/atmos'), ('file', f'gfs.t{hh}z.pgrb2.0p25.f{fh:03d}')]
    q += [(f'var_{v}', 'on') for v in ('HGT', 'TMP', 'UGRD', 'VGRD')]
    q += [(f'lev_{p}_mb', 'on') for p in LEVELS_HPA] + [('lev_surface', 'on')]
    q += [('subregion', ''), ('leftlon', f'{box[2]:g}'), ('rightlon', f'{box[3]:g}'),
          ('toplat', f'{box[1]:g}'), ('bottomlat', f'{box[0]:g}')]
    return f'{NOMADS}/cgi-bin/filter_gfs_0p25.pl?' + urllib.parse.urlencode(q)


def make_box(lat, lon, half_lat=5.0, half_lon=7.0):
    clat, clon = round(lat * 2) / 2, round(lon * 2) / 2
    s, n = max(-89.75, clat - half_lat), min(89.75, clat + half_lat)
    return tuple(float(x) + 0.0 for x in (s, n, clon - half_lon, clon + half_lon))   # + 0.0: no "-0" in file names


def cache_name(run, fh, box, step):
    return (f'gfs_{run:%Y%m%d%H}_f{fh[0]:03d}-{fh[-1]:03d}s{step}_'
            f'{box[0]:g}_{box[1]:g}_{box[2]:g}_{box[3]:g}.habw')


def download(run, fh, box, step, cache, progress=None, log=print):
    """Fetch every step, decode, pack and write <cache>/<name>.habw. Returns the path."""
    os.makedirs(cache, exist_ok=True)
    name = cache_name(run, fh, box, step)
    path = os.path.join(cache, name)
    if os.path.exists(path):
        return path
    nk, frames, terrain, geo = len(LEVELS_HPA), [], None, None
    for i, f in enumerate(fh):
        if progress:
            progress(i, len(fh), f'GFS {run:%d %b %H}Z +{f} h')
        blob = None
        for attempt in range(3):
            try:
                with _open(filter_url(run, f, box), timeout=90) as r:
                    blob = r.read()
                break
            except urllib.error.HTTPError as e:
                if e.code in (403, 429) or attempt == 2:
                    raise ConnectionError(f'NOMADS answered HTTP {e.code} for +{f} h')
            except (urllib.error.URLError, TimeoutError, ConnectionError):
                if attempt == 2:
                    raise
            time.sleep(3 + 4 * attempt)
        if not blob or not blob.startswith(b'GRIB'):
            raise ConnectionError(f'NOMADS sent no GRIB data for +{f} h: {blob[:120]!r}')
        frame = np.zeros((nk, 4) + ((geo['ny'], geo['nx']) if geo else (0, 0)), dtype=np.float32)
        got = set()
        for m in grib2_messages(blob):
            if geo is None:
                geo = {k: m[k] for k in ('lat0', 'lon0', 'dlat', 'dlon', 'ny', 'nx')}
                frame = np.zeros((nk, 4, geo['ny'], geo['nx']), dtype=np.float32)
            if (m['ny'], m['nx']) != (geo['ny'], geo['nx']):
                raise GribError('fields on different grids')
            if m['key'] == 'gh' and m['level'][0] == 'surface':
                if terrain is None:
                    terrain = m['values']
                continue
            if m['key'] is None or m['level'][0] != 'hpa' or m['level'][1] not in LEVELS_HPA:
                continue
            k, j = LEVELS_HPA.index(m['level'][1]), VARS.index(m['key'])
            frame[k, j] = m['values']
            got.add((k, j))
        if len(got) != nk * 4:
            missing = [f'{VARS[j]}@{LEVELS_HPA[k]}' for k in range(nk) for j in range(4) if (k, j) not in got]
            raise GribError(f'+{f} h is missing {len(missing)} fields, e.g. {", ".join(missing[:6])}')
        if np.isnan(frame).any():
            raise GribError(f'+{f} h has missing values')
        frames.append(frame)
        time.sleep(0.6)                       # be polite to NOMADS
    if progress:
        progress(len(fh), len(fh), 'packing')
    if terrain is None:
        terrain = np.zeros((geo['ny'], geo['nx']), dtype=np.float32)
    arr = np.stack(frames)                                   # [t][k][var][y][x]
    arr = np.transpose(arr, (0, 3, 4, 1, 2))                 # -> [t][y][x][k][var]
    raw = np.empty(arr.shape, dtype=np.int16)
    for j in range(4):
        raw[..., j] = np.clip(np.round((arr[..., j] - OFFSET[j]) / SCALE[j]), -32768, 32767)
    ter = np.clip(np.round(terrain), -500, 9000).astype(np.int16)
    valid0 = run + dt.timedelta(hours=fh[0])
    head = dict(format=1, source='NOAA GFS 0.25° (NOMADS)', run=run.strftime('%Y-%m-%dT%H:%MZ'),
                run_epoch=int(run.timestamp()), t0=int(valid0.timestamp()), dt=step * 3600, steps=fh,
                nt=len(fh), ny=geo['ny'], nx=geo['nx'], nk=nk, lat0=geo['lat0'], lon0=geo['lon0'],
                dlat=geo['dlat'], dlon=geo['dlon'], levels_hpa=LEVELS_HPA, vars=VARS, scale=SCALE, offset=OFFSET,
                box=list(box), created=dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%dT%H:%MZ'),
                layout='terrain int16 [y][x] m, then int16 [t][y][x][level][var]; rows south->north, columns west->east')
    write_habw(path, head, ter, raw)
    log(f'Wind file written: {name} ({os.path.getsize(path) / 1e6:.1f} MB)')
    return path


def write_habw(path, head, terrain, raw):
    hj = json.dumps(head).encode()
    pad = (-(12 + len(hj))) % 8
    tmp = path + '.part'
    with open(tmp, 'wb') as f:
        f.write(MAGIC + struct.pack('<II', 1, len(hj) + pad) + hj + b' ' * pad)
        f.write(terrain.astype('<i2').tobytes())
        f.write(raw.astype('<i2').tobytes())
    os.replace(tmp, path)


def read_head(path):
    with open(path, 'rb') as f:
        if f.read(4) != MAGIC:
            raise ValueError('not a HAB-RECON wind file')
        _, n = struct.unpack('<II', f.read(8))
        return json.loads(f.read(n))


def cached(cache):
    out = []
    for p in glob.glob(os.path.join(cache, '*.habw')):
        try:
            h = read_head(p)
            h['file'] = os.path.basename(p)
            out.append(h)
        except Exception:
            pass
    return sorted(out, key=lambda h: h['run_epoch'], reverse=True)


def covers(h, lat, lon, t_from=None, t_to=None, margin=1.0):
    lat1 = h['lat0'] + (h['ny'] - 1) * h['dlat']
    dlon = (lon - h['lon0']) % 360
    inside = h['lat0'] + margin <= lat <= lat1 - margin and margin <= dlon <= (h['nx'] - 1) * h['dlon'] - margin
    if not inside or t_from is None:
        return inside
    return h['t0'] <= t_from and h['t0'] + (h['nt'] - 1) * h['dt'] >= t_to


# =====================================================================
#  Background jobs for the vision server (/api/wind)
# =====================================================================
JOBS, JLOCK = {}, threading.Lock()


def request(q, cache):
    """GET /api/wind?lat&lon&start(epoch s, UTC)&hours&step -> state dict (poll until 'ready')."""
    lat, lon = float(q.get('lat', 42.6567)), float(q.get('lon', 23.3553))
    start = int(float(q.get('start') or time.time()))
    start -= start % 3600
    hours = max(3, min(168, int(float(q.get('hours', 48)))))
    step = 1 if q.get('step') == '1' else 3
    box = make_box(lat, lon)
    key = f'{box}|{start}|{hours}|{step}'
    with JLOCK:
        job = JOBS.get(key)
        age = time.time() - job['t'] if job else 0
        if job and (job['state'] == 'running' or (job['state'] == 'ready' and age < 3600) or age < 20):
            return {k: v for k, v in job.items() if k != 't'}
        job = JOBS[key] = dict(state='running', done=0, total=0, msg='looking for the newest GFS run', t=time.time())
    threading.Thread(target=_job, args=(job, lat, lon, start, hours, step, box, cache), daemon=True).start()
    return {k: v for k, v in job.items() if k != 't'}


def _job(job, lat, lon, start, hours, step, box, cache):
    def prog(i, n, msg):
        job.update(done=i, total=n, msg=msg)
    t_from = start
    t_to = start + hours * 3600
    try:
        run, fh = find_run(dt.datetime.fromtimestamp(start, dt.timezone.utc), hours, step, log=lambda s: None)
        name = cache_name(run, fh, box, step)
        if not os.path.exists(os.path.join(cache, name)):
            # a cached file of the same run that already covers the request is just as good
            for h in cached(cache):
                if h['run_epoch'] == int(run.timestamp()) and covers(h, lat, lon, t_from, t_to):
                    name = h['file']
                    break
        path = os.path.join(cache, name)
        if not os.path.exists(path):
            path = download(run, fh, box, step, cache, prog, log=lambda s: None)
        h = read_head(path)
        job.update(state='ready', file=os.path.basename(path), head=h, note='', msg='', done=job.get('total', 0))
    except Exception as e:                      # offline or NOMADS trouble: fall back to the cache
        why = f'{type(e).__name__}: {e}'
        best = None
        for h in cached(cache):
            if covers(h, lat, lon, t_from, t_to):
                best, note = h, f'offline - using the cached GFS {h["run"]} run ({why})'
                break
        if best is None:
            for h in cached(cache):
                if covers(h, lat, lon):
                    best, note = h, f'offline - the cached GFS {h["run"]} run does not cover this launch time ({why})'
                    break
        if best:
            job.update(state='ready', file=best['file'], head=best, note=note, msg='')
        else:
            job.update(state='error', msg=f'could not get GFS data: {why}. With internet, download once '
                                          f'(or run vision\\download_wind.bat) and it is kept for offline use.')
    job['t'] = time.time()


def wind_file_path(name, cache):
    if not name or '/' in name or '\\' in name or not name.endswith('.habw'):
        return None
    p = os.path.join(cache, name)
    return p if os.path.isfile(p) else None


# =====================================================================
#  Command line
# =====================================================================
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='Download the NOAA GFS wind forecast for the HAB-RECON simulation')
    ap.add_argument('--lat', type=float, default=42.6567)
    ap.add_argument('--lon', type=float, default=23.3553)
    ap.add_argument('--start', help='first time you need, UTC, "YYYY-MM-DD HH:MM" (default: now)')
    ap.add_argument('--hours', type=int, default=48, help='how many hours after --start (default 48, max 168)')
    ap.add_argument('--step', type=int, choices=[1, 3], default=3, help='hours between steps (1 only up to +120 h)')
    ap.add_argument('--cache', default=os.path.join(here, 'wind_cache'))
    ap.add_argument('--decode', metavar='FILE', help='list the fields in a GRIB2 file and exit')
    a = ap.parse_args()
    if a.decode:
        for m in grib2_messages(open(a.decode, 'rb').read()):
            v = m['values']
            print(f"{str(m['key'] or m['param']):>12} {m['level'][0]:>8} {m['level'][1]:>7}  valid {m['valid']:%Y-%m-%d %H:%MZ}  "
                  f"{m['ny']}x{m['nx']} from {m['lat0']:.2f}N {m['lon0']:.2f}E  min {np.nanmin(v):9.2f} max {np.nanmax(v):9.2f}")
        return
    start = (dt.datetime.strptime(a.start, '%Y-%m-%d %H:%M').replace(tzinfo=dt.timezone.utc) if a.start
             else dt.datetime.now(dt.timezone.utc).replace(minute=0, second=0, microsecond=0))
    box = make_box(a.lat, a.lon)
    print(f'Box {box[0]:g}..{box[1]:g} N, {box[2]:g}..{box[3]:g} E · from {start:%d %b %H:%M} UTC for {a.hours} h')
    run, fh = find_run(start, min(168, a.hours), a.step)
    print(f'GFS run {run:%Y-%m-%d %H}Z, forecast hours {fh[0]}..{fh[-1]} every {a.step} h ({len(fh)} files)')
    path = download(run, fh, box, a.step, a.cache,
                    progress=lambda i, n, m: print(f'  {i}/{n}  {m}', flush=True))
    print(f'Done: {path}\nThe dashboard uses it automatically ("Wind data: GFS forecast"), also offline.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
