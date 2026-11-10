#!/usr/bin/env python3
"""Build docs/lidar_statistics.html from the Gazebo lidar runs.

  python3 docs/build_lidar_statistics.py runs/lidar_gazebo_<date>/lidar_stats.json

scripts/lidar_stats_gazebo.sh runs the laps, `lidar_stats --runs` makes the JSON, and
this script calls it last.  Every number on the page comes from that JSON, the lidar
profiles in falcon_description/config/lidars and the track CSV in
falcon_bringup/tracks.  Videos: docs/videos/lidar_<lidar>.mp4 when they exist.
"""

import csv
import datetime
import html
import shutil
import statistics
import subprocess
import json
import math
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROFILES = os.path.join(ROOT, 'falcon_ws/src/falcon_description/config/lidars')
TRACKS = os.path.join(ROOT, 'falcon_ws/src/falcon_bringup/tracks')
OUT = os.path.join(ROOT, 'docs', 'lidar_statistics.html')
# The docs hub (falcon_driverless, next to this repo): a copy of the page goes in its
# root so tools/build_docs.py links it from index.html, videos in figures/lidar_videos.
HUB = os.environ.get('DDT_DOCS', os.path.join(os.path.dirname(ROOT), 'falcon_driverless'))

# Display order and fixed colour slot (validated categorical palette, slots 1-8).
LIDARS = [
    ('hesai_atx', 'Hesai ATX', 1),
    ('ouster_os2_128', 'Ouster OS2-128', 6),
    ('ouster_os1_128', 'Ouster OS1-128', 5),
    ('ouster_os1_64', 'Ouster OS1-64', 4),
    ('ouster_os1_32', 'Ouster OS1-32', 3),
    ('hesai_xt32', 'Hesai XT32', 8),
    ('robosense_helios_1615', 'RoboSense Helios 1615', 7),
    ('robosense_helios_5515', 'RoboSense Helios 5515', None),
    ('velodyne_vlp16', 'Velodyne VLP-16 (owned)', 2),
]
NAME = {k: n for k, n, _ in LIDARS}
SLOT = {k: s for k, _, s in LIDARS}
OS1 = ('90 m at 10% reflectivity', 'https://autonomoustuff.com/-/media/Images/Hexagon/Hexagon-Core/'
       'autonomousstuff/pdf/ouster-os1-rev7-datasheet.ashx', 'Ouster OS1 Rev7 datasheet')
RANGE_SRC = {
    'hesai_atx': ('230 m at 10% reflectivity', 'https://www.hesaitech.com/product/atx/', 'Hesai, ATX'),
    'ouster_os2_128': ('200 m at 10% reflectivity', 'https://autonomoustuff.com/-/media/Images/Hexagon/'
                       'Hexagon-Core/autonomousstuff/pdf/ouster-os2-rev7-datasheet.ashx',
                       'Ouster OS2 Rev7 datasheet'),
    'ouster_os1_128': OS1, 'ouster_os1_64': OS1, 'ouster_os1_32': OS1,
    'hesai_xt32': ('80 m at 10% reflectivity', 'https://hesaitech.com/product/xt32', 'Hesai, XT32'),
    'robosense_helios_1615': ('110 m at 10% reflectivity', 'https://robosense.ai/rslidar/Helios',
                              'RoboSense, Helios'),
    'robosense_helios_5515': ('110 m at 10% reflectivity', 'https://robosense.ai/rslidar/Helios',
                              'RoboSense, Helios'),
    'velodyne_vlp16': ('100 m, no reflectivity given', 'https://cdn.shopify.com/s/files/1/1750/5061/'
                       'files/63-9229_Rev-H_Puck__Datasheet_Web.pdf', 'Velodyne Puck datasheet'),
}
RULES = [('3:1', '3 points', 'something is there'),
         ('10:2', '10 points on 2 channels', 'a cluster with a height profile'),
         ('20:3', '20 points on 3 channels', 'enough for a cone-shape fit')]
MAIN_RULE = '10:2'
TRACE_SET = ('hesai_atx', 'ouster_os2_128', 'ouster_os1_64', 'velodyne_vlp16')
SWEEP_SET = ('hesai_atx', 'ouster_os2_128', 'ouster_os1_128', 'ouster_os1_64', 'ouster_os1_32',
             'velodyne_vlp16')
SIS = 'https://github.com/FS-AI/FS-AI_API/blob/main/Docs/ADS-DV_Software_Interface_Specification_v4.0.pdf'
API_C = 'https://github.com/FS-AI/FS-AI_API/blob/main/FS-AI_API/fs-ai_api.c'
MTI8 = 'https://www.xsens.com/hubfs/MTi%20product%20leaflet_07022025_MTi-8.pdf'
FS_AI = 'https://www.imeche.org/docs/default-source/1-oscar/formula-student/2026/rules/fs-ai-2026-rules-v1.pdf?sfvrsn=2'
CONE_SPEC = ('https://www.imeche.org/docs/default-source/1-oscar/formula-student/2021/forms/ai/'
             'fs-ai-dynamic-events-setup-and-cones-specification.pdf?sfvrsn=2')
ADS_SPEC = ('https://github.com/FS-AI/FS-AI_ADS-DV_Documentation/blob/bb8f1624936f13c10a3406ea060088f2ffb4b908/'
            'FS-AI%20ADS-DV%20Specifications%20%26%20Dimensions.md?plain=1#L64-L79')
MAP_TOL = '0.25'
# Laps with this noise seed and later ran with the wider body box of vehicles/ads_dv.yaml.  Before
# it the box stopped 2 mm short of the simulator car's tail, and the Ouster OS1's detector found
# a "cone" 1.4 m behind the car in every scan.
BOX_WIDENED_SEED = 6


def lap_seed(run):
    """Noise seed of a lap: <lidar>_s<seed> for a repeat, 1 for the recorded lap."""
    tail = run.rsplit('_s', 1)
    return int(tail[1]) if len(tail) == 2 and tail[1].isdigit() else 1


def tail_note(keys, laps_of, names):
    """A sentence on false on-track cones that only laps before the body-box fix mapped, or ''."""
    hit = {}
    for k in keys:
        laps = laps_of(k)
        bad = sorted(lap_seed(r['run']) for r in laps if r.get('false_on_track'))
        later = [r for r in laps if lap_seed(r['run']) >= BOX_WIDENED_SEED]
        if bad:
            if max(bad) >= BOX_WIDENED_SEED or not later:
                return ''
            hit[k] = bad
    if not hit:
        return ''
    seeds = lambda b: ', '.join(map(str, b[:-1])) + ' and ' + str(b[-1]) if len(b) > 1 else str(b[0])  # noqa: E731
    return ('The false cones on the track (' + '; '.join(f'the {names[k]}, seeds {seeds(b)}' for k, b in hit.items()) +
            ') are the car\'s own tail. On those laps the box that drops the car\'s own points stopped 2 mm short of '
            'the tail of the simulator\'s car, range noise put tail points just behind it, and the detector found a cone '
            f'1.4 m behind the car. The laps from seed {BOX_WIDENED_SEED} on ran with the box about 0.1 m past the body '
            'all round and mapped none. ')


def drift_note(keys, var, names):
    """Variant laps: lidars whose pose drifted past 1 m in a lap, and laps whose loop did not close."""
    worst = {k: max(r['max'] for r in var(k)['laps']) for k in keys}
    drift = sorted((k for k in keys if worst[k] > 1.0), key=lambda k: -worst[k])
    if not drift:
        return ''
    reach = max(var(k)['ahead_p50']['median'] for k in drift)
    txt = (f'The lidars whose map held no more than {math.ceil(reach)} m ahead see each cone over only a few metres of '
           'driving, and between cones the pose follows the wheel speeds and the IMU, so their error grew over the lap, '
           'past 1 m in the worst lap: ' + ', '.join(f'{names[k]} {worst[k]:.2f} m' for k in drift) + '. '
           'Back at the start line the SLAM sees the first cones again and pulls the pose and the map back, by about '
           '1 m at most (the loop closes). ')
    for k in keys:
        for r in var(k)['laps']:
            if r.get('false_on_track'):
                txt += (f'In the {names[k]}\'s seed {lap_seed(r["run"])} lap the drift was past the 1 m the SLAM allows '
                        'between a detection and its mapped cone, so the loop did not close, and the '
                        f'{r["false_on_track"]} cones it mapped with the drifted pose lie more than 1 m from where they '
                        'are: those are its false cones. ')
    return txt


def variant_label(name):
    if name.startswith('detector') and name[8:].isdigit():
        return f'detector needs {name[8:]} points'
    return name.replace('_', ' ')


def tiers(keys, laps_of):
    """Split keys (best first) where a lidar's best lap is below every lap of the group
    above it: (rank label per key).  laps_of(k) -> that lidar's per-lap values."""
    out, start, floor = {}, 0, None
    for i, k in enumerate(keys):
        v = laps_of(k)
        if floor is not None and max(v) < floor:
            start = i
            floor = None
        floor = min(v) if floor is None else min(floor, min(v))
        out[k] = start
    groups = {}
    for k, s_ in out.items():
        groups.setdefault(s_, []).append(k)
    lab = {}
    for s_, ks in groups.items():
        r = f'{s_ + 1}' if len(ks) == 1 else f'{s_ + 1}–{s_ + len(ks)}'
        for k in ks:
            lab[k] = r
    return lab


def esc(s):
    return html.escape(str(s))


def fmt(v, d=1):
    if v is None:
        return '–'
    if isinstance(v, (int, float)) and d == 0:
        return f'{v:.0f}'
    return f'{v:.{d}f}' if isinstance(v, float) else str(v)


def link(text, url):
    return f'<a href="{esc(url)}" target="_blank" rel="noopener">{esc(text)}</a>'


def profile(key):
    with open(os.path.join(PROFILES, key + '.yaml')) as f:
        p = yaml.safe_load(f)
    v = p['vertical']
    if 'uniform' in v:
        u = v['uniform']
        n, lo, hi = u['channels'], u['min_deg'], u['max_deg']
        spacing, even = (hi - lo) / (n - 1), 'even'
    else:
        a = sorted(v['angles_deg'])
        n, lo, hi = len(a), a[0], a[-1]
        spacing, even = min(b - c for b, c in zip(a[1:], a[:-1])), 'uneven'
    h = p['horizontal']
    return {'name': p['name'], 'channels': n, 'vfov': f'{lo:+g}° to {hi:+g}°, {even}',
            'vspacing': spacing, 'hfov': h['fov_deg'], 'hstep': h['fov_deg'] / h['samples'],
            'range': p['range']['max_m']}


def track_cones(track):
    path = os.path.join(TRACKS, f'{track}.csv')
    if not os.path.isfile(path):
        return []
    with open(path) as f:
        return [(r['tag'], float(r['x']), float(r['y'])) for r in csv.DictReader(f)]


def track_features(track):
    if track != 'lidar_test_track':
        return []
    sys.path.insert(0, TRACKS)
    sys.dont_write_bytecode = True
    import make_lidar_test_track as m
    return m.features()


# ---------------------------------------------------------------- charts (inline SVG)
def dist_chart(cid, title, desc, rows, xmax, digits=1, unit='m', step=None):
    """One row per lidar: min-max line, 5th-95th bar, 25th-75th box, median tick."""
    W, lh, left, right, top = 680, 34, 182, 118, 22
    pw = W - left - right
    H = top + lh * len(rows) + 34
    sx = lambda v: left + pw * min(v, xmax) / xmax  # noqa: E731
    out = [f'<svg viewBox="0 0 {W} {H}" class="chart" role="img" aria-labelledby="{cid}-t {cid}-d">'
           f'<title id="{cid}-t">{esc(title)}</title><desc id="{cid}-d">{esc(desc)}</desc>']
    step = step or (50 if xmax > 250 else 25 if xmax > 120 else 20 if xmax > 80 else 10)
    for i in range(int(round(xmax / step)) + 1):
        t = i * step
        x = sx(t)
        out.append(f'<line class="grid" x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{H - 26}"/>'
                   f'<text class="tick" x="{x:.1f}" y="{H - 10}" text-anchor="middle">{t:g}</text>')
    out.append(f'<text class="tick" x="{left + pw}" y="{H - 10}" text-anchor="end" dx="40">{esc(unit)}</text>')
    out.append(f'<text class="tick" x="{left + pw + 10}" y="{top - 9}">5th · median</text>')
    for i, (lab, k, tip) in enumerate(rows):
        y = top + i * lh + lh / 2
        out.append(f'<g class="hit" tabindex="0"><title>{esc(tip)}</title>'
                   f'<rect class="hitbox" x="0" y="{y - lh / 2}" width="{W}" height="{lh}"/>'
                   f'<text class="lab" x="{left - 8}" y="{y + 4}" text-anchor="end">{esc(lab)}</text>'
                   f'<line class="whisk" x1="{sx(k["min"]):.1f}" y1="{y}" x2="{sx(k["max"]):.1f}" y2="{y}"/>'
                   f'<rect class="p90" x="{sx(k["p5"]):.1f}" y="{y - 4}" width="{max(sx(k["p95"]) - sx(k["p5"]), 1.5):.1f}" height="8" rx="2"/>'
                   f'<rect class="iqr" x="{sx(k["p25"]):.1f}" y="{y - 8}" width="{max(sx(k["p75"]) - sx(k["p25"]), 2):.1f}" height="16" rx="3"/>'
                   f'<line class="med" x1="{sx(k["p50"]):.1f}" y1="{y - 11}" x2="{sx(k["p50"]):.1f}" y2="{y + 11}"/>'
                   f'<text class="val" x="{left + pw + 10}" y="{y + 4}">{fmt(k["p5"], digits)} · '
                   f'{fmt(k["p50"], digits)}</text></g>')
    out.append('</svg>')
    key = ('<div class="dkey"><span><i class="k-whisk"></i>lowest to highest</span>'
           '<span><i class="k-p90"></i>5th to 95th percentile</span>'
           '<span><i class="k-iqr"></i>25th to 75th</span><span><i class="k-med"></i>median</span></div>')
    return key + ''.join(out)


def trace_chart(cid, title, desc, series, lap, features, ylabel='known ahead, m', ystep=None,
                digits=1, unit='m', names=None):
    """A value along the lap per series; crosshair tooltip in JS (data in a JSON block).
    series: (key, s_m, values, done) with key a lidar (colour from SLOT) or one of `names`
    ({key: (label, css colour)})."""
    W, H, left, right, top, bottom = 680, 360, 46, 118, 30, 40
    pw, ph = W - left - right, H - top - bottom
    top_v = max(max(v) for _, _, v, _ in series)
    if ystep is None:
        ymax = max(50, math.ceil(top_v / 50) * 50)
        ystep = 50 if ymax > 150 else 25
    else:
        ymax = max(ystep, math.ceil(top_v / ystep - 1e-9) * ystep)
    names = names or {}
    nm = lambda k: names[k][0] if k in names else NAME[k].replace(' (owned)', '')  # noqa: E731
    col = lambda k: names[k][1] if k in names else f'var(--c{SLOT[k]})'  # noqa: E731
    sx = lambda s: left + pw * s / lap  # noqa: E731
    sy = lambda v: top + ph * (1 - min(v, ymax) / ymax)  # noqa: E731
    out = [f'<div class="trace" id="{cid}"><svg viewBox="0 0 {W} {H}" class="chart" role="img" '
           f'aria-labelledby="{cid}-t {cid}-d"><title id="{cid}-t">{esc(title)}</title><desc id="{cid}-d">{esc(desc)}</desc>']
    for i in range(int(round(ymax / ystep)) + 1):
        t = i * ystep
        y = sy(t)
        out.append(f'<line class="grid" x1="{left}" y1="{y:.1f}" x2="{left + pw}" y2="{y:.1f}"/>'
                   f'<text class="tick" x="{left - 6}" y="{y + 4:.1f}" text-anchor="end">{t:g}</text>')
    for t in range(0, int(lap) + 1, 50):
        out.append(f'<text class="tick" x="{sx(t):.1f}" y="{top + ph + 16}" text-anchor="middle">{t}</text>')
    for lab, s in features:
        x = sx(s)
        out.append(f'<line class="feat" x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{top + ph}"/>'
                   f'<text class="ftick" x="{x:.1f}" y="{top - 8}" text-anchor="middle">{esc(lab)}</text>')
    out.append(f'<line class="axis" x1="{left}" y1="{top + ph}" x2="{left + pw}" y2="{top + ph}"/>'
               f'<text class="tick" x="{left + pw / 2}" y="{H - 6}" text-anchor="middle">distance along lap 1, m</text>'
               f'<text class="tick" x="12" y="{top + ph / 2}" transform="rotate(-90 12 {top + ph / 2})" '
               f'text-anchor="middle">{esc(ylabel)}</text>')
    ends = []
    for key, s_m, v, done in series:
        c = col(key)
        d = 'M' + 'L'.join(f'{sx(a):.1f},{sy(b):.1f}' for a, b in zip(s_m, v))
        out.append(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="2" stroke-linejoin="round"/>')
        if done is not None:
            out.append(f'<circle cx="{sx(s_m[-1]):.1f}" cy="{sy(v[-1]):.1f}" r="4" fill="var(--bg)" '
                       f'stroke="{c}" stroke-width="2"><title>{esc(NAME[key])}: every cone of the lap '
                       f'in the map from {done:.0f} m</title></circle>')
        ends.append([sy(v[-1]), nm(key), c])
    ends.sort()
    for i in range(1, len(ends)):
        ends[i][0] = max(ends[i][0], ends[i - 1][0] + 13)
    for i in range(len(ends) - 1, -1, -1):                # keep the stack inside the plot
        ends[i][0] = min(ends[i][0], top + ph - 4 - 13 * (len(ends) - 1 - i))
    for y, lab, c in ends:
        out.append(f'<text class="lab" x="{left + pw + 8}" y="{y + 4:.1f}">{esc(lab)}</text>')
    out.append(f'<line class="xh" x1="0" y1="{top}" x2="0" y2="{top + ph}" visibility="hidden"/>'
               f'<rect class="hitbox" x="{left}" y="{top}" width="{pw}" height="{ph}"/></svg>'
               f'<div class="tip" hidden></div></div>')
    data = {'left': left, 'pw': pw, 'W': W, 'lap': lap, 'digits': digits, 'unit': unit,
            'series': [{'name': nm(k), 'colour': col(k), 's': s_m, 'v': v,
                        'done': done} for k, s_m, v, done in series]}
    out.append(f'<script type="application/json" id="{cid}-data">{json.dumps(data)}</script>')
    legend = ''.join(f'<span style="--c:{col(k)}">{esc(names[k][0] if k in names else NAME[k])}</span>'
                     for k, _, _, _ in series)
    return f'<div class="legend">{legend}</div>' + ''.join(out)


def line_chart(cid, title, desc, series, xs, xlabel, ylabel, ymax, logy=False, xlog=False, ticks=None,
               ylo=1.0):
    """series: (key, [(x, y)]); colour from SLOT, labels at the line ends."""
    W, H, left, right, top, bottom = 680, 340, 52, 150, 12, 44
    pw, ph = W - left - right, H - top - bottom
    x0, x1 = xs[0], xs[-1]
    fx = (lambda v: math.log10(v)) if xlog else (lambda v: v)
    sx = lambda v: left + pw * (fx(v) - fx(x0)) / (fx(x1) - fx(x0))  # noqa: E731
    if logy:
        span = math.log10(ymax) - math.log10(ylo)
        sy = lambda v: top + ph * (1 - (math.log10(max(v, ylo)) - math.log10(ylo)) / span)  # noqa: E731
        yt = [t for t in (0.1, 0.3, 1, 3, 10, 30, 100, 300, 1000, 3000) if ylo <= t <= ymax]
    else:
        sy = lambda v: top + ph * (1 - v / ymax)  # noqa: E731
        yt = list(range(0, int(ymax) + 1, 10 if ymax <= 60 else 20 if ymax <= 120 else 50))
    out = [f'<svg viewBox="0 0 {W} {H}" class="chart" role="img" aria-labelledby="{cid}-t {cid}-d">'
           f'<title id="{cid}-t">{esc(title)}</title><desc id="{cid}-d">{esc(desc)}</desc>']
    for t in yt:
        y = sy(t)
        out.append(f'<line class="grid" x1="{left}" y1="{y:.1f}" x2="{left + pw}" y2="{y:.1f}"/>'
                   f'<text class="tick" x="{left - 6}" y="{y + 4:.1f}" text-anchor="end">{t:g}</text>')
    for t in (ticks or xs):
        out.append(f'<text class="tick" x="{sx(t):.1f}" y="{top + ph + 16}" text-anchor="middle">{t:g}</text>')
    out.append(f'<line class="axis" x1="{left}" y1="{top + ph}" x2="{left + pw}" y2="{top + ph}"/>'
               f'<text class="tick" x="{left + pw / 2}" y="{H - 6}" text-anchor="middle">{esc(xlabel)}</text>'
               f'<text class="tick" x="12" y="{top + ph / 2}" transform="rotate(-90 12 {top + ph / 2})" '
               f'text-anchor="middle">{esc(ylabel)}</text>')
    ends = []
    for key, pts in series:
        pts = [(x, y) for x, y in pts if y is not None and (not logy or y >= ylo)]
        if not pts:
            continue
        c = f'var(--c{SLOT[key]})'
        d = 'M' + 'L'.join(f'{sx(x):.1f},{sy(y):.1f}' for x, y in pts)
        out.append(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="2"/>')
        for x, y in pts:
            out.append(f'<g class="hit" tabindex="0"><title>{esc(NAME[key])}: {fmt(y)} at {x:g} '
                       f'({esc(xlabel)})</title><circle class="hitbox" cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="9"/>'
                       f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="4" fill="{c}" stroke="var(--bg)" '
                       f'stroke-width="2"/></g>')
        if pts[-1][0] == x1:                              # lines that stop early: legend only
            ends.append([sy(pts[-1][1]), NAME[key].replace(' (owned)', ''), c])
    ends.sort()
    for i in range(1, len(ends)):
        ends[i][0] = max(ends[i][0], ends[i - 1][0] + 13)
    for i in range(len(ends) - 1, -1, -1):                # keep the stack inside the plot
        ends[i][0] = min(ends[i][0], top + ph - 4 - 13 * (len(ends) - 1 - i))
    for y, lab, c in ends:
        out.append(f'<text class="lab" x="{left + pw + 8}" y="{y + 4:.1f}">{esc(lab)}</text>')
    out.append('</svg>')
    legend = ''.join(f'<span style="--c:var(--c{SLOT[k]})">{esc(NAME[k])}</span>' for k, _ in series)
    return f'<div class="legend">{legend}</div>' + ''.join(out)


def track_svg(cones, path_xy, features):
    """The lap to scale: cones, the ATX run's driven line, labelled features."""
    xs = [c[1] for c in cones] + [p[0] for p in path_xy]
    ys = [c[2] for c in cones] + [p[1] for p in path_xy]
    x0, x1, y0, y1 = min(xs) - 6, max(xs) + 6, min(ys) - 6, max(ys) + 6
    W = 680
    k = W / (x1 - x0)
    H = (y1 - y0) * k
    P = lambda x, y: (f'{(x - x0) * k:.1f}', f'{(y1 - y) * k:.1f}')  # noqa: E731
    out = [f'<svg viewBox="0 0 {W} {H:.0f}" class="chart track" role="img" aria-labelledby="tk-t">'
           f'<title id="tk-t">The test lap to scale, with the car\'s driven line</title>']
    if path_xy:
        d = 'M' + 'L'.join(','.join(P(x, y)) for x, y in path_xy)
        out.append(f'<path class="drive" d="{d}"/>')
    cls = {'blue': 'cb', 'yellow': 'cy', 'orange': 'co', 'big_orange': 'co'}
    for tag, x, y in cones:
        if tag in cls:
            cx, cy = P(x, y)
            out.append(f'<circle class="{cls[tag]}" cx="{cx}" cy="{cy}" r="{3.2 if tag == "big_orange" else 2.6}"/>')
    car = [c for c in cones if c[0] == 'car_start']
    if car:
        cx, cy = P(car[0][1], car[0][2])
        out.append(f'<path class="car" d="M{cx},{cy}m-7,-5l14,5l-14,5z"/>'
                   f'<text class="lab" x="{cx}" y="{float(cy) + 30:.1f}" text-anchor="middle">start</text>')
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    for lab, x, y in features:
        if lab in ('start straight', 'back straight'):
            continue
        if lab in LABEL_AT:
            ox, oy, anchor = LABEL_AT[lab]
            px, py = P(x + ox, y + oy)
        else:
            dx, dy = x - mx, y - my
            n = math.hypot(dx, dy) or 1
            px, py = P(x - dx / n * 11, y - dy / n * 11)     # inside the loop
            anchor = 'middle' if abs(dx) < abs(dy) else ('end' if dx > 0 else 'start')
        out.append(f'<text class="lab" x="{px}" y="{py}" text-anchor="{anchor}">{esc(lab)}</text>')
    sbx, sby = P(x0 + 4, y0 + 2.5)
    out.append(f'<line class="axis" x1="{sbx}" y1="{sby}" x2="{float(sbx) + 20 * k:.1f}" y2="{sby}" stroke-width="2"/>'
               f'<text class="tick" x="{float(sbx) + 10 * k:.1f}" y="{float(sby) - 6:.1f}" text-anchor="middle">20 m</text>')
    out.append('</svg>')
    return ''.join(out)


# Label offsets (m) and anchors for lidar_test_track's features, so labels sit beside them.
LABEL_AT = {'hairpin 1': (5.0, -1.0, 'start'), 'hairpin 2': (-4.0, -1.0, 'end'),
            'constant turn': (-6.0, 2.0, 'end'), 'chicane': (-6.0, -1.0, 'end'),
            'hairpin 3': (-4.0, -1.0, 'end'), 'hairpin 4': (4.0, -1.0, 'start'),
            'decreasing radius': (5.0, -1.0, 'start')}


def nearest_s(trace, x, y):
    d = [(a - x) ** 2 + (b - y) ** 2 for a, b in zip(trace['x_m'], trace['y_m'])]
    return trace['s_m'][d.index(min(d))]


# ---------------------------------------------------------------- page
def main():
    data = json.load(open(sys.argv[1]))
    res = {r['lidar']: r for r in data['results']}
    lidars = [k for k, _, _ in LIDARS if k in res]
    if not lidars:
        raise SystemExit('no known lidars in ' + sys.argv[1])
    atx, vlp = 'hesai_atx', 'velodyne_vlp16'
    first = res[lidars[0]]
    track, lap = first['track'], first['lap_m']
    spinners = [s for s in lidars if s != atx]

    def R(key, rule=MAIN_RULE):
        return res[key]['rules'][rule]

    def ka(key, q='p50', rule=MAIN_RULE):
        return R(key, rule)['known_ahead_m'][q]

    def fd(key, q='p50', rule=MAIN_RULE):
        v = R(key, rule)['first_detect_m']
        return v[q] if v else None

    best = max(spinners, key=lambda s: ka(s)) if spinners else None
    cones = track_cones(track)
    feats = track_features(track)
    tr_atx = R(atx)['trace'] if atx in res else R(lidars[0])['trace']
    feat_s = [(lab, nearest_s(tr_atx, x, y)) for lab, x, y in feats
              if lab not in ('start straight', 'back straight')]
    short = {'hairpin 1': 'H1', 'hairpin 2': 'H2', 'hairpin 3': 'H3', 'hairpin 4': 'H4',
             'constant turn': 'turn', 'chicane': 'chicane', 'decreasing radius': 'dec. radius'}
    incomplete = [s for s in lidars if not res[s].get('complete', True)]
    speed = first.get('target_speed')
    n_cones = first['cones']

    p = []
    p.append(f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lidar Statistics</title>
<meta name="description" content="Gazebo statistics for the ADS-DV lidar choice: {len(lidars)} lidars drive the same rule-legal lap with four hairpins, every frame of each simulated point cloud is counted on every cone, and how far ahead each one knows the track is shown in full, uncapped, with the screen recordings; the car drives on its own pose (wheel speeds, MTi-8 IMU and cone SLAM on the lidar under test), and its localisation error is compared per lidar too.">
<meta name="doc-section" content="Findings">
<meta name="date" content="{datetime.date.today().isoformat()}">
<style>
:root{{color-scheme:light;--bg:#fcfcfb;--panel:#f3f2ef;--ink:#0b0b0b;--ink2:#52514e;--ink3:#76756f;
--rule:#dddcd6;--s1:#2a78d6;--code:#efeee9;--best:#e8f1fc;--cb:#2a5bd6;--cy:#d9a400;--co:#eb6834;--drive:#9a998f;
--c1:#2a78d6;--c2:#eb6834;--c3:#1baf7a;--c4:#eda100;--c5:#e87ba4;--c6:#008300;--c7:#6250d6;--c8:#e34948}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{color-scheme:dark;--bg:#1a1a19;--panel:#232321;
--ink:#f0efec;--ink2:#c3c2b7;--ink3:#9a998f;--rule:#3a3936;--s1:#3987e5;--code:#2a2a27;--best:#1f2c3d;
--cb:#5b8af0;--cy:#e8bb2a;--co:#f07a48;--drive:#76756f;
--c1:#3987e5;--c2:#d95926;--c3:#199e70;--c4:#c98500;--c5:#d55181;--c6:#008300;--c7:#9085e9;--c8:#e66767}}}}
:root[data-theme="dark"]{{color-scheme:dark;--bg:#1a1a19;--panel:#232321;--ink:#f0efec;--ink2:#c3c2b7;--ink3:#9a998f;
--rule:#3a3936;--s1:#3987e5;--code:#2a2a27;--best:#1f2c3d;--cb:#5b8af0;--cy:#e8bb2a;--co:#f07a48;--drive:#76756f;
--c1:#3987e5;--c2:#d95926;--c3:#199e70;--c4:#c98500;--c5:#d55181;--c6:#008300;--c7:#9085e9;--c8:#e66767}}
*{{box-sizing:border-box}}
body{{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}}
main{{max-width:980px;margin:0 auto;padding:32px 16px 64px}}
a{{color:var(--s1)}}
h1{{font-size:2rem;margin:0 0 4px;letter-spacing:-.01em}}
h2{{font-size:1.35rem;margin:44px 0 10px;padding-top:14px;border-top:1px solid var(--rule)}}
h3{{font-size:1.05rem;margin:26px 0 8px}}
.meta{{color:var(--ink2);margin:0 0 22px;font-size:.95rem}} .lede{{font-size:1.1rem;color:var(--ink2);margin:0 0 6px}}
.key{{background:var(--panel);border-radius:10px;padding:14px 18px;margin:18px 0}}
.key h2{{border:0;margin:0 0 6px;padding:0;font-size:1.05rem}} .key ul{{margin:0;padding-left:20px}} .key li{{margin:6px 0}}
.warn{{border-left:4px solid var(--c8);background:var(--panel);padding:10px 14px;border-radius:6px;margin:14px 0}}
code{{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:.88em;background:var(--code);padding:1px 5px;border-radius:4px}}
.tw{{overflow-x:auto;margin:10px 0 6px;border:1px solid var(--rule);border-radius:8px}}
table{{border-collapse:collapse;width:100%;font-size:.84rem}}
th,td{{padding:6px 8px;text-align:left;border-bottom:1px solid var(--rule);vertical-align:top}}
th{{color:var(--ink2);font-weight:600;background:var(--panel)}} tbody tr:last-child td{{border-bottom:0}}
td.num,th.num{{text-align:right;font-variant-numeric:tabular-nums;white-space:nowrap}}
tr.atx td{{background:var(--best);font-weight:600}}
.note{{color:var(--ink2);font-size:.92rem}}
figure{{margin:16px 0}} figcaption{{color:var(--ink2);font-size:.9rem;margin-top:6px}}
.legend,.dkey{{display:flex;gap:6px 16px;font-size:.85rem;color:var(--ink2);margin:4px 0 6px;flex-wrap:wrap}}
.legend span::before{{content:"";display:inline-block;width:12px;height:3px;border-radius:2px;margin-right:6px;vertical-align:3px;background:var(--c)}}
.dkey i{{display:inline-block;margin-right:6px;vertical-align:middle;border-radius:2px}}
.k-whisk{{width:16px;height:2px;background:var(--ink3)}} .k-p90{{width:16px;height:8px;background:var(--s1);opacity:.35}}
.k-iqr{{width:16px;height:14px;background:var(--s1)}} .k-med{{width:2px;height:16px;background:var(--ink)}}
svg.chart{{width:100%;height:auto;display:block}}
svg .grid{{stroke:var(--rule);stroke-width:1}} svg .axis{{stroke:var(--ink3);stroke-width:1}}
svg .ref{{stroke:var(--ink3);stroke-width:1;stroke-dasharray:4 3}} svg .feat{{stroke:var(--rule);stroke-width:1;stroke-dasharray:3 3}}
svg .tick{{fill:var(--ink3);font-size:11px}} svg .ftick{{fill:var(--ink2);font-size:10.5px}}
svg .lab{{fill:var(--ink);font-size:12px}} svg .val{{fill:var(--ink2);font-size:11.5px;font-variant-numeric:tabular-nums}}
svg .whisk{{stroke:var(--ink3);stroke-width:1.5}} svg .p90{{fill:var(--s1);opacity:.35}} svg .iqr{{fill:var(--s1)}}
svg .med{{stroke:var(--ink);stroke-width:2}}
svg .hitbox{{fill:transparent}} svg .hit:hover .iqr,svg .hit:focus .iqr{{opacity:.8}} svg .hit:focus{{outline:none}}
svg .xh{{stroke:var(--ink3);stroke-width:1}}
svg.track .cb{{fill:var(--cb)}} svg.track .cy{{fill:var(--cy)}} svg.track .co{{fill:var(--co)}}
svg.track .drive{{fill:none;stroke:var(--drive);stroke-width:1.2;stroke-dasharray:5 4}} svg.track .car{{fill:var(--ink)}}
svg.diag .box{{fill:var(--panel);stroke:var(--ink3);stroke-width:1.2}}
svg.diag .t{{fill:var(--ink);font-size:13px;font-weight:600}} svg.diag .s{{fill:var(--ink2);font-size:11.5px}}
svg.diag .ln{{stroke:var(--ink2);stroke-width:1.5;fill:none}} svg.diag .ah{{fill:var(--ink2)}}
.trace{{position:relative}} .tip{{position:absolute;top:8px;pointer-events:none;background:var(--bg);border:1px solid var(--rule);
border-radius:6px;padding:6px 9px;font-size:.8rem;line-height:1.45;box-shadow:0 2px 8px rgba(0,0,0,.12);white-space:nowrap}}
.tip b{{display:block;color:var(--ink2);font-weight:600}} .tip i{{display:inline-block;width:10px;height:3px;border-radius:2px;margin-right:6px;vertical-align:3px}}
video{{width:100%;border-radius:8px;background:#000}} .vid{{margin:14px 0 22px}}
.missing{{border:1px dashed var(--rule);border-radius:8px;padding:12px 14px;color:var(--ink2);font-size:.92rem}}
.tabs{{display:flex;gap:6px;flex-wrap:wrap;margin:8px 0}} .tabs button{{font:inherit;font-size:.85rem;padding:4px 10px;border-radius:6px;border:1px solid var(--rule);background:var(--bg);color:var(--ink);cursor:pointer}}
.tabs button[aria-pressed="true"]{{background:var(--panel);border-color:var(--ink3);font-weight:600}}
</style></head>
<body><main>
<h1>Lidar Statistics</h1>
<p class="lede">Which lidar should the ADS-DV carry? Each candidate drove the same rule-legal lap in Gazebo, and
every frame of its simulated point cloud was counted on every cone. The car drove on its own pose, so each lidar is
also judged on how well the car knew where it was.</p>
<p class="meta">Gazebo 11 with GPU-rendered lidars (RTX 5060) · ADS-DV, {fmt(speed)} m/s target speed ·
{esc(track)}: {lap:.0f} m driven, {n_cones} cones, four hairpins · {len(lidars)} lidars · one lap each</p>
''')
    if any(r.get('source') != 'gazebo' for r in data['results']):
        p.append('<div class="warn"><b>Preview data.</b> These numbers are a ray-cast of each lidar along one Gazebo lap '
                 'of this track, not Gazebo\'s own point clouds. They are replaced when '
                 '<code>scripts/lidar_stats_gazebo.sh</code> has run every lidar in Gazebo.</div>')
    if incomplete:
        p.append('<div class="warn"><b>Incomplete laps:</b> ' + ', '.join(esc(NAME[s]) for s in incomplete) +
                 ' stopped before the end of the lap; their numbers cover only the part driven.</div>')

    # ---- results box
    items = []
    if atx in res and best:
        items.append(
            f'<li><b>The ATX knows the track furthest ahead.</b> For 95% of lap 1 it had at least '
            f'{fmt(ka(atx, "p5"))} m of track ahead fully in its map, and a median of {fmt(ka(atx))} m. '
            f'The best spinning lidar, the {esc(NAME[best])}, had {fmt(ka(best, "p5"))} m and {fmt(ka(best))} m; '
            + (f'the VLP-16 we own had {fmt(ka(vlp, "p5"))} m and {fmt(ka(vlp))} m.' if vlp in res else '') + '</li>')
        done = R(atx).get('map_complete_at_m')
        if done is not None:
            others = sorted((R(s).get('map_complete_at_m') or lap, s) for s in spinners)
            items.append(f'<li><b>The ATX had every cone of the lap in its map {lap - done:.0f} m before the finish</b> '
                         f'({done:.0f} m into the {lap:.0f} m lap). The earliest spinning lidar, the {esc(NAME[others[0][1]])}, '
                         f'got there {lap - others[0][0]:.0f} m before the finish'
                         + (f'; the VLP-16 {lap - (R(vlp).get("map_complete_at_m") or lap):.0f} m before.' if vlp in res else '.')
                         + '</li>')
        items.append(
            f'<li><b>It sees each cone from further away.</b> Half the cones first met the rule more than '
            f'{fmt(fd(atx))} m from the ATX (90th percentile {fmt(fd(atx, "p90"))} m). The {esc(NAME[best])}: '
            f'{fmt(fd(best))} m; ' + (f'the VLP-16: {fmt(fd(vlp))} m.' if vlp in res else '') + '</li>')
        wins = [rk for rk, _, _ in RULES if all(ka(atx, rule=rk) >= ka(s, rule=rk) and
                                                (fd(atx, rule=rk) or 0) >= (fd(s, rule=rk) or 0)
                                                for s in spinners)]
        if len(wins) == len(RULES):
            items.append('<li><b>The detection rule does not change the order at the top.</b> Under all three rules '
                         'the ATX has the longest median known-ahead distance and first-detection range.</li>')
        else:
            items.append('<li><b>The detection rule matters:</b> the ATX leads both medians under '
                         + (', '.join(wins) or 'none') + ' of the three rules; see the table.</li>')
        items.append(
            '<li><b>Why: beam spacing.</b> The ATX spaces its 256 channels about 0.08° apart, 25 times closer than the '
            'VLP-16 and about 12 times closer than a 32-channel lidar, so a cone far ahead still crosses several of its beams. '
            'Its 120° view costs little: the cones beside and behind the car are already in the map.</li>')
    locs = [k for k in lidars if res[k].get('localization')]
    LO = lambda k: res[k]['localization']  # noqa: E731
    if locs:
        def lap_p95(k, q='median'):
            lp = LO(k).get('laps') or {}
            return lp['p95'][q] if lp.get('n', 0) > 1 else LO(k)['slam']['position_m']['p95']
        by = sorted(locs, key=lap_p95)
        b0, bl = by[0], by[-1]
        many = all((LO(k).get('laps') or {}).get('n', 0) > 1 for k in locs)
        n_laps = min((LO(k).get('laps') or {}).get('n', 1) for k in locs)
        dr_max = max(LO(k)['dead_reckoning']['position_m']['max'] for k in locs)
        txt = (f'<li><b>Localisation: the car drove on its own pose, and with every lidar it knew it to within '
               f'centimetres.</b> ' + (f'Over {n_laps} laps per lidar, the' if many else 'The') +
               f' 95th percentile of the position error was {lap_p95(b0):.2f} m with the {esc(NAME[b0])} '
               f'and {lap_p95(bl):.2f} m with the {esc(NAME[bl])}' + (' (medians over the laps)' if many else '') +
               f'; wheel speeds and the IMU alone drifted up to {dr_max:.2f} m. ')
        if many:
            # tiers() ranks higher-is-better values: pass the errors negated.
            lab = tiers(by, lambda k: [-r['p95'] for r in LO(k)['laps']['laps']])
            grp = {}
            for k in by:
                grp.setdefault(lab[k], []).append(k)
            if len(grp) > 1:
                txt += ('Ranked, with lidars sharing a place where their laps overlap: ' +
                        '; '.join(f'{r_}: ' + ', '.join(esc(NAME[k]) for k in ks) for r_, ks in grp.items()) +
                        '. A lidar ranks below a group only if even its best lap was worse than every lap in that group. ')
            else:
                txt += 'The lidars overlap from lap to lap, so their order is not settled by these runs. '
        txt += '<a href="#loc">Localisation</a> has the distributions and the lap-to-lap spread.</li>'
        items.append(txt)
        mlap = [k for k in locs if (LO(k).get('laps') or {}).get('n_map', 0) > 0]
        if mlap:
            ML = lambda k: LO(k)['laps']  # noqa: E731
            sc = lambda k, q='ahead_p50': ML(k)[q]['median']  # noqa: E731
            ranked = sorted(mlap, key=lambda k: -sc(k))
            rank = tiers(ranked, lambda k: [r['ahead_p50'] for r in ML(k)['laps'] if r.get('ahead_p50') is not None])
            top = [k for k in ranked if rank[k] == rank[ranked[0]]]
            txt = (f'<li><b>Map building: the {esc(NAME[ranked[0]])}\'s map held the most track ahead.</b> '
                   f'Median over the lap of how far ahead the SLAM map had every cone within {MAP_TOL} m of where it is: '
                   + ', '.join(f'{esc(NAME[k].split(" (")[0])} {sc(k):.0f} m' for k in ranked) +
                   f' (medians over {min(ML(k)["n_map"] for k in mlap)} laps each). ')
            if len(top) > 1:
                txt += ('The laps of ' + ', '.join(f'the {esc(NAME[k])}' for k in top) +
                        ' overlap, so they share first place. ')
            alt = {q: sorted(mlap, key=lambda k: -sc(k, q)) for q in ('ahead_tight_p50', 'ahead_loose_p50')}
            if all(v[0] == ranked[0] for v in alt.values()):
                txt += 'The top lidar is the same at 0.1 m and 0.5 m. '
            else:
                txt += ('At 0.1 m the top is the ' + esc(NAME[alt['ahead_tight_p50'][0]]) + ', at 0.5 m the ' +
                        esc(NAME[alt['ahead_loose_p50'][0]]) + '. ')
            acc = sorted(mlap, key=lambda k: sc(k, 'map_error_p95'))
            txt += (f'The finished map, which later laps drive on, was most accurate with the {esc(NAME[acc[0]])} '
                    f'({sc(acc[0], "map_error_p95"):.2f} m, 95th percentile) and least with the {esc(NAME[acc[-1]])} '
                    f'({sc(acc[-1], "map_error_p95"):.2f} m). ')
            for vn in sorted({v for k in mlap for v in (LO(k).get('variants') or {})}):
                vk = [k for k in mlap if vn in (LO(k).get('variants') or {})]
                vs = lambda k: LO(k)['variants'][vn]['ahead_p50']['median']  # noqa: E731
                vr = sorted(vk, key=lambda k: -vs(k))
                txt += (f'With a stricter detector ({esc(variant_label(vn))}): ' +
                        ', '.join(f'{esc(NAME[k].split(" (")[0])} {vs(k):.0f} m' for k in vr) + '. ')
                vp = lambda k: LO(k)['variants'][vn]['p95']['median']  # noqa: E731
                lo_ = sorted(vk, key=vp)
                wmax = max(r['max'] for k in vk for r in LO(k)['variants'][vn]['laps'])
                txt += (f'Its position error, 95th percentile, ran from {vp(lo_[0]):.2f} m '
                        f'({esc(NAME[lo_[0]].split(" (")[0])}) to {vp(lo_[-1]):.2f} m ({esc(NAME[lo_[-1]].split(" (")[0])}), '
                        f'medians over the laps; the lidars that mapped only a few metres ahead drifted, up to {wmax:.1f} m '
                        'in a lap (table 14). ')
            txt += '<a href="#map">Map building</a> has the score per lidar.</li>'
            items.append(txt)
    p.append('<section class="key"><h2>Results</h2><ul>' + ''.join(items) + '</ul></section>')

    # ---- the track
    p.append('<h2>The test lap</h2>')
    if cones:
        p.append('<figure>' + track_svg(cones, list(zip(tr_atx['x_m'], tr_atx['y_m'])), feats) +
                 f'<figcaption>{esc(track)}, to scale. Blue cones mark the left edge, yellow the right; the dashed '
                 'line is the path the car drove (the ATX run). It drives anticlockwise from the start.</figcaption></figure>')
    p.append(f'''<p>The lap is built to the FS-AI 2026 trackdrive rules: straights no longer than 80 m, constant turns up
to 50 m across, hairpins at least 9 m outside diameter, and a track at least 3 m wide ({link("FS-AI 2026 rules, D8.1, p. 33", FS_AI + "#page=33")}).
Cones are at most 5 m apart in the driving direction and closer in corners
({link("cones specification, p. 2", CONE_SPEC + "#page=2")}). It has a 60 m straight into a hairpin, two pairs of
hairpins back to back, a 30 m constant turn, a chicane and a decreasing-radius turn; {lap:.0f} m in all.</p>
<p>The hairpins are 13 m across, not the 9 m minimum. The ADS-DV can steer at most ±21°, so the tightest circle its
outer front tyre can turn is 9.9 m across: it cannot drive a 9 m hairpin at all. It did drive 11 m hairpins in the
simulator, but touched cones on the way round, and a lap that touches cones is not a fair test. The script that builds
the lap is <code>falcon_bringup/tracks/make_lidar_test_track.py</code>.</p>''')

    # ---- what is measured
    p.append(f'''<h2>What is measured</h2>
<div class="tw"><table><thead><tr><th>Statistic</th><th>What it is</th><th>Why it matters</th></tr></thead><tbody>
<tr><td><b>Detection rule</b></td><td>A cone counts as detected in a frame when the lidar puts at least <i>N</i> points on it,
from at least <i>K</i> different channels. Three rules: 3 points; 10 points on 2 channels (used for the headline numbers);
20 points on 3 channels.</td>
<td>3 points only says "something is there". Two or more channels give the cluster a height, so it can be told from
ground noise; a cone-shape fit wants more. Showing all three stops the result depending on one choice.</td></tr>
<tr><td><b>Known ahead</b></td><td>At every half metre of lap 1: how far along the driven path every cone is already in
the map. The first cone ahead that has not met the rule in any frame so far ends it. <b>Nothing is capped.</b> Reported
as the full distribution: lowest, 5th, 25th, median, 75th, 95th percentile, highest. It covers the lap up to the point
where every cone of the lap is in the map, which is reported on its own ("map complete"): near the end of lap 1 the only
cones left ahead are the ones seen at the start, so from there on the whole lap is known with any lidar.</td>
<td>On lap 1, and in autocross, the car cannot plan past the first cone it has not seen. That point limits how fast it may
drive into what lies beyond. The 5th percentile is the distance it had for 95% of the lap; the lowest value is the
single worst moment.</td></tr>
<tr><td><b>First detection</b></td><td>The distance from the lidar at which each cone first met the rule. 10th percentile,
median, 90th percentile and furthest, over every cone.</td>
<td>How far ahead a cone becomes a landmark.</td></tr>
<tr><td><b>Points on a cone</b></td><td>Mean points on a cone in view, by range band.</td><td>The raw signal behind every other number.</td></tr>
</tbody></table></div>''')

    # ---- method
    p.append(f'''<h2>How the numbers were made</h2>
<figure><svg class="diag" viewBox="0 0 760 200" role="img" aria-labelledby="dm-t"><title id="dm-t">Pipeline: Gazebo lap, simulated lidar, points counted on each cone, statistics</title>
<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path class="ah" d="M0,0L10,5L0,10z"/></marker></defs>
<rect class="box" x="10" y="20" width="160" height="80" rx="8"/><text class="t" x="90" y="46" text-anchor="middle">Gazebo lap</text>
<text class="s" x="90" y="66" text-anchor="middle">ADS-DV, Falcon planner</text><text class="s" x="90" y="84" text-anchor="middle">and pure pursuit</text>
<rect class="box" x="205" y="20" width="160" height="80" rx="8"/><text class="t" x="285" y="46" text-anchor="middle">Simulated lidar</text>
<text class="s" x="285" y="66" text-anchor="middle">every beam of the profile,</text><text class="s" x="285" y="84" text-anchor="middle">GPU-rendered, 10 Hz</text>
<rect class="box" x="400" y="20" width="160" height="80" rx="8"/><text class="t" x="480" y="46" text-anchor="middle">Count on each cone</text>
<text class="s" x="480" y="66" text-anchor="middle">points and channels,</text><text class="s" x="480" y="84" text-anchor="middle">every frame (frames.npz)</text>
<rect class="box" x="595" y="20" width="155" height="80" rx="8"/><text class="t" x="672" y="46" text-anchor="middle">Statistics</text>
<text class="s" x="672" y="66" text-anchor="middle">per rule, per lidar</text><text class="s" x="672" y="84" text-anchor="middle">lidar_stats.json</text>
<path class="ln" d="M170,60H203" marker-end="url(#ah)"/><path class="ln" d="M365,60H398" marker-end="url(#ah)"/><path class="ln" d="M560,60H593" marker-end="url(#ah)"/>
<text class="s" x="90" y="128" text-anchor="middle">steers on simulated</text><text class="s" x="90" y="144" text-anchor="middle">cones (20 m, 180°): the</text><text class="s" x="90" y="160" text-anchor="middle">same drive for every lidar</text>
<text class="s" x="285" y="128" text-anchor="middle">cones hide cones,</text><text class="s" x="285" y="144" text-anchor="middle">the car hides beams,</text><text class="s" x="285" y="160" text-anchor="middle">2 cm range noise</text>
<text class="s" x="480" y="128" text-anchor="middle">points within 0.25 m</text><text class="s" x="480" y="144" text-anchor="middle">of a cone's axis, above</text><text class="s" x="480" y="160" text-anchor="middle">the ground</text>
<text class="s" x="672" y="128" text-anchor="middle">any rule applied</text><text class="s" x="672" y="144" text-anchor="middle">afterwards to the</text><text class="s" x="672" y="160" text-anchor="middle">same frames</text>
</svg></figure>
<ol>
<li><b>Drive.</b> The car is the ADS-DV model (1.53 m wheelbase, ±21° steering) in the EUFS Gazebo simulator. The
stack's own centreline planner and pure pursuit drive one lap at up to {fmt(speed)} m/s, on the car's own pose
estimate (see <a href="#loc">Localisation</a>). They steer on the cones the simulator reports within 20 m and 180°,
not on the lidar under test, so <b>every lidar gets nearly the same drive</b>: the lines differ only by the
localisation error, centimetres.</li>
<li><b>Simulate the lidar.</b> Gazebo renders every beam of the lidar's profile on the GPU, ten times a second, from its
mount on the car, and publishes the point cloud a real lidar would. Every lidar gets all ten frames: the nodes share
the clouds through 32 MB shared-memory segments. With the default transport a 3 MB ATX frame went as UDP fragments,
and most ATX frames were lost. Near cones hide far ones, the car's body hides
beams, and each range gets 2 cm of Gaussian noise. Each lidar's range is the figure its datasheet gives for a 10%
reflective target (table below).</li>
<li><b>Count.</b> The <code>cone_coverage</code> node takes each cloud, places it on the map using the simulator's
true car pose (not the car's estimate, so only the lidar is judged here), and counts the points within 0.25 m of each cone's axis and above the ground, and the channels they came
from. It keeps every frame and writes them to <code>frames.npz</code> when the lap ends.</li>
<li><b>Statistics.</b> <code>lidar_stats --runs</code> applies each detection rule to those frames and computes the
statistics above. <code>scripts/lidar_stats_gazebo.sh</code> runs all of it, one lidar after another, and records each
run; the videos below are those recordings.</li>
</ol>
<p>Mounts: the ATX on the front sensor plate, 0.57 m up (the 2026 CarMaker model's lidar height); every spinning lidar on
a short post above it, 0.70 m up, level.</p>
<h3>Lidar profiles</h3>''')
    rows = []
    for s in lidars:
        q = profile(s)
        txt, url, src = RANGE_SRC[s]
        z = (res[s].get('lidar_mount') or {}).get('z')
        rows.append('<tr' + (' class="atx"' if s == atx else '') + f'><td>{esc(NAME[s])}</td>'
                    f'<td class="num">{fmt(z, 2)} m</td><td class="num">{q["channels"]}</td><td>{esc(q["vfov"])}</td>'
                    f'<td class="num">{q["vspacing"]:.2f}°</td><td class="num">{q["hfov"]:g}°</td>'
                    f'<td class="num">{q["hstep"]:.2f}°</td>'
                    f'<td>{q["range"]:g} m{"" if "10%" in txt else " (" + esc(txt.split(", ", 1)[-1]) + ")"}: '
                    f'{link(src, url)}</td></tr>')
    p.append('<div class="tw"><table><thead><tr><th>Lidar</th><th class="num">Height</th><th class="num">Channels</th>'
             '<th>Vertical view</th><th class="num">Finest vertical spacing</th><th class="num">Horizontal view</th>'
             '<th class="num">Column step</th><th>Range, 10% target (datasheet)</th></tr></thead><tbody>'
             + ''.join(rows) + '</tbody></table></div>'
             '<p class="note">Profiles are the YAML files in <code>falcon_description/config/lidars/</code>. The ATX profile '
             'spreads its channels evenly over ±10°; the real unit packs them closer near the middle (0.05° at the finest), '
             'which would help it further.</p>')

    # ---- results
    p.append('<h2>Results</h2>')
    rows = []
    kmax = max(R(s)['known_ahead_m']['max'] for s in lidars)
    kmax = math.ceil(kmax / (25 if kmax > 120 else 10)) * (25 if kmax > 120 else 10)
    for s in lidars:
        k = R(s)['known_ahead_m']
        rows.append((NAME[s].replace(' (owned)', ''), k,
                     f'{NAME[s]}: lowest {fmt(k["min"])} m, 5th pct {fmt(k["p5"])}, 25th {fmt(k["p25"])}, '
                     f'median {fmt(k["p50"])}, 75th {fmt(k["p75"])}, 95th {fmt(k["p95"])}, highest {fmt(k["max"])} m'))
    p.append('<h3>1. Track known ahead over lap 1</h3><figure>' +
             dist_chart('c1', 'Known ahead over lap 1, distribution per lidar',
                        f'One row per lidar, metres along the track, rule {RULES[1][1]}; table below has the numbers.',
                        rows, kmax) +
             f'<figcaption>Rule: {RULES[1][1]}. One value every half metre of lap 1, up to the point where that lidar had '
             'every cone of the lap in its map; nothing capped.</figcaption></figure>')

    ser = []
    for s in TRACE_SET:
        if s not in res:
            continue
        t, done = R(s)['trace'], R(s).get('map_complete_at_m')
        keep = [i for i, a in enumerate(t['s_m']) if done is None or a < done]
        ser.append((s, [t['s_m'][i] for i in keep], [t['known_ahead_m'][i] for i in keep], done))
    if ser:
        p.append('<h3>2. Known ahead along the lap</h3><figure>' +
                 trace_chart('c2', 'Known ahead at each point of lap 1',
                             'Lines per lidar; x is the distance driven, y the track ahead already in the map.',
                             ser, lap, [(short.get(lab, lab), s) for lab, s in feat_s]) +
                 '<figcaption>Hover for every lidar\'s value at that point. Labels at the top mark the hairpins (H1 to H4), '
                 'the constant turn, the chicane and the decreasing-radius turn. Each drop is the car reaching the first cone it '
                 'has not detected yet. A line ends (open circle) where that lidar had every cone of the lap in its map.</figcaption></figure>')

    th = [str(d) for d in (10, 20, 30, 50, 75, 100, 150, 200) if d < lap]
    body = ''.join('<tr' + (' class="atx"' if s == atx else '') + f'><td>{esc(NAME[s])}</td>' +
                   ''.join(f'<td class="num">{100 * R(s)["known_ahead_at_least"][d]:.0f}%</td>' for d in th) + '</tr>'
                   for s in lidars)
    p.append('<h3>3. Share of the lap with at least this much track known ahead</h3>'
             '<div class="tw"><table><thead><tr><th>Lidar</th>' +
             ''.join(f'<th class="num">≥ {d} m</th>' for d in th) + f'</tr></thead><tbody>{body}</tbody></table></div>'
             f'<p class="note">Rule: {RULES[1][1]}. Read a row as: for this share of lap 1 (up to the point where that '
             'lidar\'s map is complete), the car knew at least that much track ahead.</p>')

    sw = [s for s in SWEEP_SET if s in res]
    xs = [int(x) for x in res[sw[0]]['point_sweep']] if sw else []
    if sw:
        series = [(s, [(x, (res[s]['point_sweep'][str(x)]['first_detect_m'] or {}).get('p50')) for x in xs])
                  for s in sw]
        top = max((y or 0) for _, pts in series for _, y in pts)
        behind = [x for x in xs if atx in res and any(
            (y2 or 0) > ((res[atx]['point_sweep'][str(x)]['first_detect_m'] or {}).get('p50') or 0)
            for s2, pts in series if s2 != atx for xx, y2 in pts if xx == x)]
        if atx not in res or not behind:
            sweep_caption = 'A stricter rule shortens every lidar\'s reach. The ATX stays furthest at every threshold.'
        else:
            lead = [x for x in xs if x > max(behind)]
            sweep_caption = ('A stricter rule shortens every lidar\'s reach. '
                             + (f'From {lead[0]} points up the ATX is furthest. ' if lead else '')
                             + f'At {", ".join(str(x) for x in behind)} point{"s" if len(behind) > 1 else ""} a 360° lidar '
                             'with long range edges ahead: at such loose rules the limit is how far away the track\'s cones are, '
                             'and a 360° lidar can see cones across the whole track, where the ATX sees only the 120° ahead.')
        p.append('<h3>4. First detection against the detection rule</h3><figure>' +
                 line_chart('c4', 'Median first-detection range against minimum points per cone',
                            'Lines per lidar; x is the minimum points on a cone, y the median first detection in metres.',
                            series, xs, 'minimum points on a cone (any channels)', 'median first detection, m',
                            max(20, math.ceil(top / 20) * 20), xlog=True) +
                 f'<figcaption>{sweep_caption}</figcaption></figure>')
        bands = [b['range_m'] for b in res[sw[0]]['points_by_range']]
        mids = [(float(b.split('-')[0]) + float(b.split('-')[1])) / 2 for b in bands]
        series = []
        for s in sw:
            pr = {b['range_m']: b['mean_points'] for b in res[s]['points_by_range']}
            series.append((s, [(m, pr.get(b)) for m, b in zip(mids, bands)]))
        top = max((y or 0) for _, pts in series for _, y in pts)
        far = max(m for m, b in zip(mids, bands) if any(pr for s2, pts in series for mm, pr in pts if mm == m and pr))
        ticks = [m for m in mids if m <= far]
        p.append('<h3>5. Points on a cone against range</h3><figure>' +
                 line_chart('c5', 'Mean points on a cone in view against range',
                            'Lines per lidar; x range band centre in metres (log), y mean points (log).',
                            series, ticks, 'range, m (band centre)', 'points on a cone (log)',
                            10 ** math.ceil(math.log10(max(top, 10))), logy=True, xlog=True,
                            ticks=[t for t in ticks if t in (2.5, 7.5, 17.5, 35, 62.5, 125, 225)], ylo=0.1) +
                 '<figcaption>Every frame with the cone in view. Below 1, a cone gets a point only in some frames; a line '
                 'stops where the average falls below 0.1. Points fall roughly with the square of range; the ATX starts '
                 'higher and stays higher. A dip on a lidar with widely spaced channels is a band of range where cones '
                 'fall between two of its beams.</figcaption></figure>')

    # ---- full table
    p.append('<h3>6. Every lidar, every rule</h3><div class="tabs" role="group" aria-label="Detection rule">' +
             ''.join(f'<button type="button" data-t="{rk}" aria-pressed="{str(rk == MAIN_RULE).lower()}">{esc(n)}</button>'
                     for rk, n, _ in RULES) + '</div>')
    for rk, n, why in RULES:
        head = ('<tr><th rowspan="2">Lidar</th><th rowspan="2" class="num">Cones detected</th>'
                '<th colspan="4">First detection, m</th><th colspan="4">Known ahead, m</th>'
                '<th rowspan="2" class="num">Map complete, m</th><th rowspan="2" class="num">Cones / frame</th></tr><tr>'
                '<th class="num">10th</th><th class="num">median</th><th class="num">90th</th><th class="num">furthest</th>'
                '<th class="num">lowest</th><th class="num">5th</th><th class="num">median</th><th class="num">95th</th></tr>')
        body = []
        for s in lidars:
            v, r = R(s, rk), res[s]
            f, k = v['first_detect_m'] or {}, v['known_ahead_m']
            body.append('<tr' + (' class="atx"' if s == atx else '') + f'><td>{esc(NAME[s])}</td>'
                        f'<td class="num">{v["detected"]}/{r["cones"]}</td>' +
                        ''.join(f'<td class="num">{fmt(f.get(q))}</td>' for q in ('p10', 'p50', 'p90', 'max')) +
                        ''.join(f'<td class="num">{fmt(k[q])}</td>' for q in ('min', 'p5', 'p50', 'p95')) +
                        f'<td class="num">{fmt(v.get("map_complete_at_m"))}</td>'
                        f'<td class="num">{fmt(v["cones_per_frame"]["p50"], 0)}</td></tr>')
        p.append(f'<div class="trk" data-t="{rk}"' + ('' if rk == MAIN_RULE else ' hidden') + '>'
                 f'<p class="note">Rule: {esc(n)} ({esc(why)}).</p>'
                 f'<div class="tw"><table><thead>{head}</thead><tbody>{"".join(body)}</tbody></table></div></div>')
    scans = ', '.join(f'{esc(NAME[s].split(" (")[0])} {res[s]["frames"]}' for s in lidars)
    p.append(f'<p class="note">Frames counted per lidar (one lap each): {scans}.</p>')

    # ---- localisation
    if locs:
        p.append(f"""<h2 id="loc">Localisation</h2>
<p>In these runs nothing in the stack read the simulator's pose. The car found its own pose from what the ADS-DV and
the IMU we plan to fit give it, and drove on it; the lidar under test fed the cone SLAM. The errors below are the
car's estimate against the simulator's true pose, which only the evaluation read.</p>
<figure><svg class="diag" viewBox="0 0 760 300" role="img" aria-labelledby="dl-t"><title id="dl-t">Localisation: CAN wheel speeds and the IMU feed the odometry EKF; the lidar's cones and the odometry feed the cone SLAM, which gives the planner its pose</title>
<rect class="box" x="10" y="20" width="170" height="64" rx="8"/><text class="t" x="95" y="46" text-anchor="middle">ADS-DV CAN</text>
<text class="s" x="95" y="66" text-anchor="middle">4 wheel rpm, steer angle</text>
<rect class="box" x="10" y="104" width="170" height="64" rx="8"/><text class="t" x="95" y="130" text-anchor="middle">MTi-8 IMU</text>
<text class="s" x="95" y="150" text-anchor="middle">gyro, accelerometer</text>
<rect class="box" x="225" y="62" width="160" height="64" rx="8"/><text class="t" x="305" y="88" text-anchor="middle">Odometry EKF</text>
<text class="s" x="305" y="108" text-anchor="middle">odom → base_footprint</text>
<rect class="box" x="10" y="188" width="170" height="64" rx="8"/><text class="t" x="95" y="214" text-anchor="middle">Lidar under test</text>
<text class="s" x="95" y="234" text-anchor="middle">Gazebo point cloud</text>
<rect class="box" x="225" y="188" width="160" height="64" rx="8"/><text class="t" x="305" y="214" text-anchor="middle">Cone detector</text>
<text class="s" x="305" y="234" text-anchor="middle">clusters → cone x, y</text>
<rect class="box" x="430" y="125" width="155" height="64" rx="8"/><text class="t" x="507" y="151" text-anchor="middle">Cone EKF-SLAM</text>
<text class="s" x="507" y="171" text-anchor="middle">map → odom, cone map</text>
<rect class="box" x="620" y="125" width="130" height="64" rx="8"/><text class="t" x="685" y="151" text-anchor="middle">Planner and</text>
<text class="t" x="685" y="169" text-anchor="middle">pure pursuit</text>
<path class="ln" d="M180,52H202V82H223" marker-end="url(#ah)"/><path class="ln" d="M180,136H202V106H223" marker-end="url(#ah)"/>
<path class="ln" d="M180,220H223" marker-end="url(#ah)"/>
<path class="ln" d="M385,94H407V145H428" marker-end="url(#ah)"/><path class="ln" d="M385,220H407V169H428" marker-end="url(#ah)"/>
<path class="ln" d="M585,157H618" marker-end="url(#ah)"/>
<text class="s" x="507" y="230" text-anchor="middle">evaluation only: the simulator's</text>
<text class="s" x="507" y="246" text-anchor="middle">true pose, map → base_footprint_gt</text>
</svg></figure>
<h3>What the car is given</h3>
<div class="tw"><table><thead><tr><th>Sensor</th><th>Simulated as</th><th>Source</th></tr></thead><tbody>
<tr><td>ADS-DV wheel speeds</td><td>All four wheels every 10 ms, as unsigned whole rpm (1 rpm = 0.026 m/s on the 0.505 m
wheel); each wheel's rolling speed at its own Ackermann angle. The tyre's true rolling radius is 1% larger than the one
the odometry assumes: an assumption, as the EUFS wheel (505 mm) and the CAD tyre (515 mm) are 2% apart.</td>
<td>VCU2AI_Speeds, {link("Software Interface Specification v4.0, p. 9", SIS + "#page=9")}; read as uint16 rpm in
{link("fs-ai_api.c", API_C)}</td></tr>
<tr><td>ADS-DV steer angle</td><td>The bicycle-model angle in 0.1° steps, every 10 ms.</td>
<td>VCU2AI_Steer, {link("SIS v4.0, p. 9", SIS + "#page=9")}; 0.1 scaling in {link("fs-ai_api.c", API_C)}</td></tr>
<tr><td>Xsens MTi-8 IMU</td><td>Gyro noise 0.003 °/s/√Hz and in-run bias 6 °/h; accelerometer 70 µg/√Hz and 40 µg; at
200 Hz. Raw rates only: the MTi's own orientation output is not used.</td>
<td>{link("MTi-8 product leaflet (Movella, Feb 2025)", MTI8)}</td></tr>
</tbody></table></div>
<p class="note">The ADS-DV's own PCAN-GPS IMU and GPS, which the FS-AI API also reads, are not used, and neither is RTK.
The recorded laps all used the same noise seed; the repeat laps (table 10) used others. The sensors come out on the topics the real drivers use
(<code>/ros_can/wheel_speeds</code> as the EUFS/Bristol <code>ros_can</code> bridge, <code>/imu/data</code> as the Xsens
driver), so the same localisation runs on the car.</p>
<h3>How it works</h3>
<ol>
<li><b>Odometry EKF</b> (<code>falcon_localization/odometry.py</code>), 200 Hz. The gyro and the accelerometer predict
heading and the body velocities. Each CAN frame then corrects them with all four wheels (each wheel's speed is the body
velocity at that wheel along its heading). It also uses a soft "the rear axle does not slide" constraint. While all
four wheels read 0 rpm at the start, the gyro's bias is learnt. On its own this is the "odometry alone" line below.</li>
<li><b>Cone detector</b> (<code>lidar_cone_detector_3d.py</code>). Points between 4 cm and 70 cm above the ground are
clustered on a 15 cm grid, and clusters the size of a cone become detections without colour. Range: the lidar's full
datasheet range. Points on the car itself are dropped by a box about 0.1 m bigger than the body, wide enough for the
front tyres at full lock; cut exactly at the tail, range noise left points of the tail outside it, and the detector
reported a cone just behind the car in every scan.</li>
<li><b>Cone EKF-SLAM</b> (<code>falcon_localization/slam.py</code>), every scan:
<ul>
<li>predicts with the odometry since the last scan;</li>
<li>pairs the detections with the mapped cones by nearest Mahalanobis distance, as one assignment, inside a χ² gate
and no more than 1 m from where the mapped cone should be. The χ² gate alone is not enough: after a lap the car's
uncertainty relative to the cones mapped at the start reaches metres, and with a detector that finds cones only within
about 6 m, a cone the map did not hold yet paired with its neighbour one cone spacing on, and the car jumped 3–5 m at
the loop closure;</li>
<li>corrects the pose and every mapped cone in one update.</li>
</ul>
A new cone joins the map once it has been seen in 3 scans, from car positions at least 1 m apart, with its sightings
staying together. Returns that move with the car never join.</li>
<li><b>Evaluation</b> (<code>falcon_evaluation/localization.py</code>). Every 0.05 s it records the estimate, the true
pose, and the odometry alone (lined up with the truth at the start). Errors are taken every 0.5 m driven, so time
standing still does not count. Like the rest of this page they are shown as full distributions, uncapped.</li>
</ol>""")
        mx = max(LO(k)['slam']['position_m']['max'] for k in locs)
        st = 0.05 if mx <= 0.3 else 0.1 if mx <= 0.6 else 0.25
        xmax = max(st, math.ceil(mx / st - 1e-9) * st)
        rows = []
        for k in locs:
            e = LO(k)['slam']['position_m']
            rows.append((NAME[k].replace(' (owned)', ''), e,
                         f'{NAME[k]}: position error lowest {e["min"]:.3f} m, 5th pct {e["p5"]:.3f}, 25th {e["p25"]:.3f}, '
                         f'median {e["p50"]:.3f}, 75th {e["p75"]:.3f}, 95th {e["p95"]:.3f}, highest {e["max"]:.3f} m'))
        dr = [LO(k)['dead_reckoning']['position_m'] for k in locs]
        p.append('<h3>7. Position error over the lap</h3><figure>' +
                 dist_chart('c7', 'Position error of the car\'s own pose over the lap, distribution per lidar',
                            'One row per lidar, metres from the true position; the table below has the numbers.',
                            rows, xmax, digits=2, step=st) +
                 f'<figcaption>The recorded lap of each lidar (the one in the videos): distance between the car\'s estimate '
                 f'and its true position, one value every 0.5 m of the lap. '
                 f'For scale: odometry alone (no lidar) reached a median of '
                 f'{statistics.median(d["p50"] for d in dr):.2f} m and at worst {max(d["max"] for d in dr):.2f} m '
                 'over the same laps.</figcaption></figure>')
        ser = [(k, LO(k)['trace']['s_m'], LO(k)['trace']['slam_position_m'], None)
               for k in TRACE_SET if k in locs]
        top = max(max(v) for _, _, v, _ in ser)
        p.append('<h3>8. Position error along the lap</h3><figure>' +
                 trace_chart('c8', 'Position error at each point of the lap',
                             'Lines per lidar and for odometry alone; x is the distance driven, y the error in metres.',
                             ser, lap, [(short.get(lab, lab), s_) for lab, s_ in feat_s],
                             ylabel='position error, m', ystep=0.05 if top <= 0.4 else 0.1, digits=2) +
                 '<figcaption>Hover for every line\'s value at that point. Each line is one lap (the recorded one); table '
                 '10 shows how much laps differ. For scale, odometry alone on the same laps drifted with distance and '
                 'swung in the hairpins, where an error in heading turns into position, up to '
                 f'{max(LO(k)["dead_reckoning"]["position_m"]["max"] for k in locs):.2f} m: off the top of this chart '
                 'by several times.</figcaption></figure>')
        body = []
        for k in locs:
            L_ = LO(k)
            e, d, m_ = L_['slam'], L_['dead_reckoning'], L_.get('map') or {}
            sc = L_.get('scans') or {}
            body.append('<tr' + (' class="atx"' if k == atx else '') + f'><td>{esc(NAME[k])}</td>' +
                        ''.join(f'<td class="num">{e["position_m"][q]:.2f}</td>' for q in ('p50', 'p95', 'max')) +
                        f'<td class="num">{e["cross_abs_m"]["p95"]:.2f}</td>'
                        f'<td class="num">{e["heading_abs_deg"]["p95"]:.2f}</td>'
                        f'<td class="num">{e["final"]["position_m"]:.2f}</td>'
                        f'<td class="num">{m_.get("mapped", 0)}/{m_.get("true_cones", 0)}</td>'
                        f'<td class="num">{fmt((m_.get("error_m") or {}).get("p95"), 2)}</td>'
                        f'<td class="num">{m_.get("false_on_track", "–")}</td>'
                        f'<td class="num">{m_.get("false_off_track", "–")}</td>'
                        f'<td class="num">{fmt((sc.get("detections") or {}).get("p50"), 0)}</td>'
                        f'<td class="num">{d["position_m"]["max"]:.2f}</td></tr>')
        p.append('<h3>9. Every lidar</h3><div class="tw"><table><thead><tr><th rowspan="2">Lidar</th>'
                 '<th colspan="3">Position error, m</th><th rowspan="2" class="num">Cross-track 95th, m</th>'
                 '<th rowspan="2" class="num">Heading 95th, °</th><th rowspan="2" class="num">At the finish, m</th>'
                 '<th colspan="4">SLAM cone map</th><th rowspan="2" class="num">Cones / scan</th>'
                 '<th rowspan="2" class="num">Odometry alone, worst, m</th></tr><tr>'
                 '<th class="num">median</th><th class="num">95th</th><th class="num">worst</th>'
                 '<th class="num">cones mapped</th><th class="num">error 95th, m</th><th class="num">false, on track</th>'
                 '<th class="num">false, off track</th></tr></thead><tbody>' + ''.join(body) + '</tbody></table></div>'
                 f'<p class="note">Every lidar delivered all its scans: '
                 f'{min((LO(k).get("scans") or {}).get("count", 0) for k in locs)}–'
                 f'{max((LO(k).get("scans") or {}).get("count", 0) for k in locs)} in the lap (10 a second). '
                 'Cross-track is sideways from the car\'s true heading, the part that puts a car over a '
                 'cone row. A mapped cone is a SLAM cone within 1 m of a true one, paired one to one. The start line\'s big '
                 'orange cones stand in pairs 0.6 m apart and are mapped as one each, so two are always unmapped. '
                 '"False, off track" are map cones more than 5 m from the driven line: ground returns that Gazebo\'s '
                 'GPU lidar puts a few centimetres above the ground near the horizon of the long-range lidars. A real lidar '
                 'does not do this, and none of them lies on the track. '
                 + tail_note(locs, lambda k: (LO(k).get('laps') or {}).get('laps') or [], {k: esc(NAME[k]) for k in locs})
                 + '</p>')
        rep = [k for k in locs if (LO(k).get('laps') or {}).get('n', 0) > 1]
        if rep:
            rng_ = lambda d: f'{d["min"]:.2f} · {d["median"]:.2f} · {d["max"]:.2f}' if d else '–'  # noqa: E731
            body = ''.join('<tr' + (' class="atx"' if k == atx else '') + f'><td>{esc(NAME[k])}</td>'
                           f'<td class="num">{LO(k)["laps"]["n"]}</td>' +
                           ''.join(f'<td class="num">{rng_(LO(k)["laps"].get(q))}</td>'
                                   for q in ('p50', 'p95', 'max', 'final', 'map_error_p95')) + '</tr>'
                           for k in rep)
            n_laps = max(LO(k)['laps']['n'] for k in rep)
            p.append('<h3>10. Lap to lap</h3><div class="tw"><table><thead><tr><th>Lidar</th>'
                     '<th class="num">Laps</th><th class="num">Median error, m</th><th class="num">95th, m</th>'
                     '<th class="num">Worst, m</th><th class="num">At the finish, m</th>'
                     '<th class="num">Map error 95th, m</th></tr></thead><tbody>' + body + '</tbody></table></div>'
                     f'<p class="note">Each lidar drove the lap {n_laps} times: the recorded lap above, and more '
                     'without the windows, each with another noise seed for the CAN and IMU (Gazebo\'s lidar noise '
                     'differs every lap anyway). Each cell is lowest · median · highest over those laps. The SLAM '
                     'is a loop: a small difference early can change what it pairs later, so one lap alone can be '
                     'lucky or unlucky. The spread shows how far to trust the order.</p>')

        # ---- map building
        mlap = [k for k in locs if (LO(k).get('laps') or {}).get('n_map', 0) > 0]
        if mlap:
            ML = lambda k: LO(k)['laps']  # noqa: E731
            per = lambda k, q: [r[q] for r in ML(k)['laps'] if r.get(q) is not None]  # noqa: E731
            ranked = sorted(mlap, key=lambda k: -ML(k)['ahead_p50']['median'])
            rank = tiers(ranked, lambda k: per(k, 'ahead_p50'))
            n_map = min(ML(k)['n_map'] for k in mlap)
            n_all = min(ML(k)['n'] for k in mlap)
            p.append(f'''<h2 id="map">Map building</h2>
<p>On lap 1 the planner drives on the SLAM's cone map, so the score for map building is how much of the track ahead
that map gets right. <b>Map ahead</b>: every 0.5 m of the lap, the distance along the track to the first cone ahead
that the map does not hold within {MAP_TOL} m of where it really is. Both positions are taken relative to the car: the map cone
from the car's own pose, the true cone from the true pose, because that is where the planner puts it. A false map cone
on the track ends it too. It is chart 1's "known ahead", which asks only whether the lidar's rays reached a cone,
after the detector, the SLAM and the car's own pose have had their say. The lidars below are ranked by its median
over the lap.</p>
<p>Why {MAP_TOL} m: the track can be 3 m wide ({link("FS-AI 2026 rules, D8.1, p. 33", FS_AI + "#page=33")}) and the ADS-DV
is 1.43 m wide ({link("ADS-DV Specifications & Dimensions, §5", ADS_SPEC)}). That leaves 0.79 m each side, and {MAP_TOL} m is
a third of it, about one cone base (228 mm, {link("cones specification, p. 2", CONE_SPEC + "#page=2")}). The table also gives the
score at 0.1 m and 0.5 m, so the ranking can be checked against the choice.</p>''')
            rows = []
            for k in mlap:
                d_ = ML(k)['map_pooled']['ahead_m']
                rows.append((NAME[k].replace(' (owned)', ''), d_,
                             f'{NAME[k]}: map ahead lowest {fmt(d_["min"])} m, 5th pct {fmt(d_["p5"])}, 25th {fmt(d_["p25"])}, '
                             f'median {fmt(d_["p50"])}, 75th {fmt(d_["p75"])}, 95th {fmt(d_["p95"])}, highest {fmt(d_["max"])} m'))
            mx = max(r[1]['max'] for r in rows)
            mx = math.ceil(mx / (25 if mx > 120 else 10)) * (25 if mx > 120 else 10)
            p.append('<h3>11. Map ahead over lap 1</h3><figure>' +
                     dist_chart('c11', 'Map ahead over lap 1, distribution per lidar',
                                f'One row per lidar, metres along the track to the first cone the map does not hold within {MAP_TOL} m; '
                                'table 13 has the numbers.', rows, mx) +
                     f'<figcaption>All {n_map} map laps of each lidar together, one value every 0.5 m, up to the point where '
                     'that lap\'s map held every cone; nothing capped.</figcaption></figure>')
            ser = []
            for k in TRACE_SET:
                if k not in mlap or not ML(k).get('map_trace'):
                    continue
                tr_ = ML(k)['map_trace']
                done = tr_.get('complete_at_m')
                keep = [i for i, a_ in enumerate(tr_['s_m']) if done is None or a_ < done]
                ser.append((k, [tr_['s_m'][i] for i in keep], [tr_['ahead_m'][i] for i in keep], done))
            if ser:
                p.append('<h3>12. Map ahead along the lap</h3><figure>' +
                         trace_chart('c12', 'Map ahead at each point of lap 1',
                                     'Lines per lidar; x is the distance driven, y the track ahead the map holds within '
                                     f'{MAP_TOL} m.', ser, lap, [(short.get(lab, lab), s_) for lab, s_ in feat_s]) +
                         '<figcaption>One map lap per lidar. Hover for every line\'s value at that point. Each drop is the car '
                         'reaching the first cone the map does not yet hold well enough; an open circle marks where the whole '
                         'map was good.</figcaption></figure>')
            rng_ = lambda d, dg=1: f'{d["min"]:.{dg}f} · {d["median"]:.{dg}f} · {d["max"]:.{dg}f}' if d else '–'  # noqa: E731
            med = lambda k, q, dg=1: fmt((ML(k).get(q) or {}).get('median'), dg)  # noqa: E731
            body = []
            for k in ranked:
                pool = ML(k)['map_pooled']
                body.append('<tr' + (' class="atx"' if k == atx else '') + f'><td class="num">{rank[k]}</td>'
                            f'<td>{esc(NAME[k])}</td><td class="num">{ML(k)["n_map"]}</td>'
                            f'<td class="num">{rng_(ML(k)["ahead_p50"])}</td>'
                            f'<td class="num">{med(k, "ahead_p5")}</td>'
                            f'<td class="num">{med(k, "ahead_tight_p50")}</td>'
                            f'<td class="num">{med(k, "ahead_loose_p50")}</td>'
                            f'<td class="num">{fmt(ka(k))}</td>'
                            f'<td class="num">{med(k, "entered_p50")}</td>'
                            f'<td class="num">{med(k, "entered_p5")}</td>'
                            f'<td class="num">{fmt((pool.get("local_error_m") or {}).get("p95"), 2)}</td>'
                            f'<td class="num">{med(k, "map_error_p95", 2)}</td>'
                            f'<td class="num">{fmt((ML(k).get("false_on_track") or {}).get("max"), 0)}</td>'
                            f'<td class="num">{fmt((ML(k).get("duplicates") or {}).get("max"), 0)}</td></tr>')
            p.append('<h3>13. Map score, every lidar</h3><div class="tw"><table><thead><tr>'
                     '<th rowspan="2" class="num">Rank</th><th rowspan="2">Lidar</th><th rowspan="2" class="num">Laps</th>'
                     f'<th colspan="4">Map ahead, m (median of the lap unless marked)</th>'
                     '<th rowspan="2" class="num">Rays reached, chart 1, median, m</th>'
                     '<th colspan="2">Cone enters the map, m ahead</th>'
                     '<th rowspan="2" class="num">Car-relative error, 95th, m</th>'
                     f'<th colspan="3">Map at the end of the lap, all {n_all} laps</th></tr><tr>'
                     f'<th class="num">{MAP_TOL} m: lowest · median · highest lap</th><th class="num">{MAP_TOL} m, 5th pct</th>'
                     '<th class="num">0.1 m</th><th class="num">0.5 m</th>'
                     '<th class="num">median</th><th class="num">5th pct</th>'
                     '<th class="num">error 95th, m</th><th class="num">false on track</th><th class="num">twice</th>'
                     '</tr></thead><tbody>' + ''.join(body) + '</tbody></table></div>'
                     f'<p class="note">Ranked by map ahead ({MAP_TOL} m, median of the lap, median over the laps). Lidars share a rank '
                     'where their laps overlap: a lidar ranks below a group only if its best lap is short of every lap in that group. '
                     'Map ahead and "cone enters the map" are medians over the map laps (the first map-ahead column also gives '
                     'the lowest and highest lap); car-relative error pools every map lap. '
                     f'Each lidar drove {n_map} laps for this with the map recorded after every scan, each with its own noise seed; '
                     f'the map at the end of the lap is recorded on all {n_all} laps: its error is the median over them, and false '
                     'cones and cones mapped twice are the most in any one lap. '
                     + tail_note(mlap, lambda k: ML(k)['laps'], {k: esc(NAME[k]) for k in mlap}) +
                     '"Rays reached" is from the recorded lap. '
                     '"Cone enters the map" is how far the car still had to drive along lap 1 to reach each cone when the map '
                     f'first held it within {MAP_TOL} m (5th pct: the latest cones). Car-relative error is the error of every mapped cone up to 30 m ahead, '
                     'as the planner sees it. The start line\'s big orange cones stand in pairs 0.6 m apart, '
                     'closer than the 1 m the SLAM keeps between its cones, so each pair counts as one cone. '
                     'The detector counts a cluster of at least 2 points standing 8 cm or more above the ground as a cone; '
                     'in Gazebo nothing but the cones is out there to be mistaken for one, which favours the lidars that '
                     'put a few points on cones far away.' + (' Table 14 repeats the test with a stricter detector.'
                     if any(LO(k).get('variants') for k in mlap) else '') + '</p>')
            vnames = sorted({v for k in mlap for v in (LO(k).get('variants') or {})})
            for vn in vnames:
                vk = [k for k in mlap if vn in (LO(k).get('variants') or {})]
                V = lambda k: LO(k)['variants'][vn]  # noqa: E731

                def pair_(k, q, how=statistics.median):
                    """q over the main laps with the variant's seeds, and over the variant's laps."""
                    runs = {r['run'] for r in V(k)['laps']}
                    a_ = [r[q] for r in ML(k)['laps'] if r['run'] in runs and r.get(q) is not None]
                    b_ = [r[q] for r in V(k)['laps'] if r.get(q) is not None]
                    return (how(a_) if a_ else None), (how(b_) if b_ else None)
                vr = sorted(vk, key=lambda k: -V(k)['ahead_p50']['median'])
                vrank = tiers(vr, lambda k: [r['ahead_p50'] for r in V(k)['laps'] if r.get('ahead_p50') is not None])
                lr = sorted(vk, key=lambda k: V(k)['p95']['median'])
                cells = lambda k, q, dg, how=statistics.median: ''.join(  # noqa: E731
                    f'<td class="num">{fmt(v, dg)}</td>' for v in pair_(k, q, how))
                body = ''.join(
                    '<tr' + (' class="atx"' if k == atx else '') + f'><td class="num">{vrank[k]}</td><td>{esc(NAME[k])}</td>'
                    f'<td class="num">{V(k)["n"]}</td>'
                    f'<td class="num">{fmt(pair_(k, "ahead_p50")[0])}</td>'
                    f'<td class="num">{rng_(V(k)["ahead_p50"])}</td>'
                    + cells(k, 'p95', 3) + f'<td class="num">{lr.index(k) + 1}</td>'
                    + cells(k, 'map_error_p95', 2) + cells(k, 'false_on_track', 0, max) + '</tr>'
                    for k in vr)
                p.append(f'<h3>14. With a stricter detector: {esc(variant_label(vn))}</h3><div class="tw"><table><thead><tr>'
                         '<th rowspan="2" class="num">Rank</th><th rowspan="2">Lidar</th><th rowspan="2" class="num">Laps</th>'
                         f'<th colspan="2">Map ahead, {MAP_TOL} m, median, m</th><th colspan="3">Position error 95th, m (median over laps)</th>'
                         '<th colspan="2">Map error 95th at the end, m</th><th colspan="2">False cones on the track, most in a lap</th></tr><tr>'
                         '<th class="num">as above</th><th class="num">stricter: lowest · median · highest lap</th>'
                         '<th class="num">as above</th><th class="num">stricter</th><th class="num">stricter, rank</th>'
                         '<th class="num">as above</th><th class="num">stricter</th>'
                         '<th class="num">as above</th><th class="num">stricter</th></tr></thead><tbody>' + body +
                         '</tbody></table></div>'
                         f'<p class="note">The same laps (same noise seeds) with the cone detector asking for more points per '
                         f'cluster ({esc(variant_label(vn))}, still 8 cm tall), close to chart 1\'s main rule. Everything else '
                         'is as above; every "as above" cell is over the laps with those same seeds. Ranked by map ahead with '
                         'the stricter detector. ' + drift_note(vk, V, {k: esc(NAME[k].split(' (')[0]) for k in vk}) + '</p>')

    # ---- videos
    p.append('<h2>Videos</h2><p>Each lidar\'s lap, recorded on the RTX 5060 laptop while the numbers above were collected: '
             'Gazebo on one side, and RViz with the lidar\'s point cloud and a marker over each cone. A cone\'s marker turns green '
             f'once it meets the {RULES[1][1]} rule.</p>')
    missing = []
    for s in lidars:
        f = os.path.join(ROOT, 'docs', 'videos', f'lidar_{s}.mp4')
        if os.path.isfile(f):
            p.append(f'<div class="vid"><h3>{esc(NAME[s])}</h3><video controls preload="metadata" '
                     f'src="videos/lidar_{s}.mp4"></video></div>')
        else:
            missing.append(s)
    if missing:
        p.append('<div class="missing">Not recorded yet: ' + ', '.join(
            f'<code>docs/videos/lidar_{s}.mp4</code>' for s in missing) + '.</div>')

    # ---- limits
    p.append(f'''<h2>Limits</h2><ul>
<li><b>Simulator cones are slimmer than real ones.</b> The EUFS cone models are 160 mm across at the base against the
rulebook's 228 mm ({link("cones specification, Table 2, p. 2", CONE_SPEC + "#page=2")}), so every lidar puts fewer points on
them than it would on a real cone.</li>
<li><b>Clean returns.</b> Every beam that meets a cone returns a point with 2 cm of range noise: no reflectivity loss,
rain, sun or dust. Real lidars lose some returns at range, more so the ones with fewer, wider beams.</li>
<li><b>Flat ground, level car.</b> No pitch from braking or bumps.</li>
<li><b>One lap, one speed, one track.</b> Each lidar drove the lap once at up to {fmt(speed)} m/s. A faster car gets
fewer frames per metre; the distances would shrink a little for every lidar.</li>
<li><b>Geometry only.</b> Cone colour, intensity and a detector's own false positives are not modelled; the rules stand in
for a detector.</li>
<li><b>Localisation: tidy sensors.</b> The car stays level, so the accelerometer never feels gravity from pitch or roll.
Wheels slip only as the simulator's tyre model allows. CAN, IMU and lidar share one clock with no delay. Each Gazebo
scan is taken at one instant, where a real spinning lidar sweeps for a tenth of a second while the car moves. The
1% tyre error is an assumption, and the IMU's noise and bias are its datasheet figures. Real numbers will be worse
for every lidar. What these runs give is the lidars side by side under the same conditions, with the lap-to-lap
spread showing which differences are real.</li>
</ul>
<script>
document.querySelectorAll('.tabs button').forEach(b => b.addEventListener('click', () => {{
  document.querySelectorAll('.tabs button').forEach(x => x.setAttribute('aria-pressed', String(x === b)));
  document.querySelectorAll('.trk').forEach(d => {{ d.hidden = d.dataset.t !== b.dataset.t; }});
}}));
document.querySelectorAll('.trace').forEach(box => {{
  const D = JSON.parse(document.getElementById(box.id + '-data').textContent);
  const svg = box.querySelector('svg'), xh = svg.querySelector('.xh'), tip = box.querySelector('.tip');
  const hit = svg.querySelector('rect.hitbox');
  function show(ev) {{
    const r = svg.getBoundingClientRect(), k = D.W / r.width;
    const x = (ev.clientX - r.left) * k, s = (x - D.left) / D.pw * D.lap;
    if (s < 0 || s > D.lap) return hide();
    xh.setAttribute('x1', x); xh.setAttribute('x2', x); xh.setAttribute('visibility', 'visible');
    let h = '<b>' + Math.round(s) + ' m into the lap</b>';
    D.series.forEach(se => {{
      let i = 0; while (i < se.s.length - 1 && se.s[i + 1] <= s) i++;
      const val = (se.done !== null && s >= se.done) ? 'every cone mapped' : se.v[i].toFixed(D.digits) + ' ' + D.unit;
      h += '<div><i style="background:' + se.colour + '"></i>' + se.name + ': ' + val + '</div>';
    }});
    tip.innerHTML = h; tip.hidden = false;
    const px = (ev.clientX - r.left);
    tip.style.left = (px > r.width / 2 ? px - tip.offsetWidth - 12 : px + 12) + 'px';
  }}
  function hide() {{ xh.setAttribute('visibility', 'hidden'); tip.hidden = true; }}
  hit.addEventListener('pointermove', show); hit.addEventListener('pointerleave', hide);
}});
</script>
</main></body></html>
''')
    page = ''.join(p)
    with open(OUT, 'w') as f:
        f.write(page)
    print('Written', OUT)
    write_hub_copy(page)


def write_hub_copy(page):
    """The same page in the docs hub, with links to the other lidar docs, then rebuild the hub."""
    if not os.path.isfile(os.path.join(HUB, 'tools', 'build_docs.py')):
        print(f'No docs hub at {HUB} (set DDT_DOCS); only {OUT} written')
        return
    vids = os.path.join(ROOT, 'docs', 'videos')
    if os.path.isdir(vids):
        dst = os.path.join(HUB, 'figures', 'lidar_videos')
        for f in sorted(os.listdir(vids)):
            if f.startswith('lidar_') and f.endswith('.mp4'):
                os.makedirs(dst, exist_ok=True)
                shutil.copy2(os.path.join(vids, f), os.path.join(dst, f))
    related = ('<p class="note">Related: <a href="lidar_resolution_and_slam.html">Lidar Resolution and SLAM</a> '
               '(the theory: rays on a cone against range, how far ahead the car must see) and '
               '<a href="lidar_coverage_and_hairpins.html">Lidar Coverage and Hairpins</a> (the track rules line by '
               'line, and a track builder held to them). Made in the <code>falcon_autonomy</code> repo by '
               '<code>docs/build_lidar_statistics.py</code>.</p>')
    page = page.replace('src="videos/lidar_', 'src="figures/lidar_videos/lidar_')
    page = page.replace('<section class="key">', related + '<section class="key">', 1)
    path = os.path.join(HUB, 'lidar_statistics.html')
    with open(path, 'w') as f:
        f.write(page)
    print('Written', path)
    subprocess.run(['python3', 'tools/build_docs.py'], cwd=HUB, check=True)


if __name__ == '__main__':
    main()
