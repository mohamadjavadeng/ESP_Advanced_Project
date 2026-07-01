#!/usr/bin/env python3
"""
update_dashboard.py -- improve the GEOMind excavator dashboard MAP (requirement #4).

The excavation dashboard is the geobox-dashboard SPA driven by an opaque
`settings` JSON (see the project notes). This tool:

  * INSPECT (default, read-only): logs in, downloads the dashboard `settings`,
    saves them to a local JSON file and prints a summary (widgets, layers, and
    the map view it found). It changes NOTHING on the server.

  * APPLY (--apply): backs up the current settings locally, then patches the map
    so the equipment shows as a marker centred on the real site near Muscat, the
    status vector layer is visible, and the basemap looks clean -- then saves.

Because editing a live dashboard is hard to undo, APPLY always writes a
`*.bak.json` backup first, and you should run the default INSPECT pass once and
eyeball the dump before using --apply.

    pip install geobox tqdm requests

    # 1) read-only: see what's there (writes dashboard_settings.json)
    python3 update_dashboard.py

    # 2) improve the map and save (writes dashboard_settings.bak.json first)
    python3 update_dashboard.py --apply

Credentials / uuids default to the project account (pdo.excavator) and the
existing `excavator1_dashboard`; override with the flags if they change.
"""

import argparse
import json
import os
import sys

try:
    from geobox import GeoboxClient
except Exception as e:                        # most likely: missing 'tqdm'
    print("cannot import geobox:", e)
    print("install deps:  pip install geobox tqdm requests")
    sys.exit(1)


# --------------------------------------------------------------------------- #
# helpers to walk the opaque settings JSON defensively
# --------------------------------------------------------------------------- #
def _main(settings):
    """The settings['main'] object (widgets/layers/pages/window live here)."""
    return settings.get("main", {}) if isinstance(settings, dict) else {}


def find_map_widget(settings):
    """Return the geoMap widget dict, or None."""
    for w in _main(settings).get("widgets", []) or []:
        if isinstance(w, dict) and w.get("classId") == "geoMap":
            return w
    return None


def summarize(settings):
    """Print a human summary of the dashboard structure."""
    m = _main(settings)
    widgets = m.get("widgets", []) or []
    layers = m.get("layers", []) or []
    queries = m.get("queries", []) or []
    print(f"  widgets ({len(widgets)}):")
    for w in widgets:
        if isinstance(w, dict):
            print(f"    - {w.get('classId'):<16} name={w.get('name')!r}")
    print(f"  layers ({len(layers)}):")
    for lyr in layers:
        if isinstance(lyr, dict):
            meta = lyr.get("metadata", {}) or {}
            print(f"    - name={lyr.get('name')!r}  type={meta.get('layer_type')}  "
                  f"features={meta.get('feature_count')}")
    print(f"  queries ({len(queries)}):")
    for q in queries:
        if isinstance(q, dict):
            print(f"    - name={q.get('name')!r}")
    mw = find_map_widget(settings)
    if mw:
        props = (mw.get("model", {}) or {}).get("properties", {}) or {}
        # print any view-ish keys so we can see how this dashboard stores center/zoom
        viewish = {k: props[k] for k in props
                   if any(s in k.lower() for s in ("center", "zoom", "lat", "lon",
                                                   "lng", "basemap", "extent", "view"))}
        print(f"  map widget properties (view-related): {viewish or '(none found)'}")
    else:
        print("  map widget: NONE FOUND (classId 'geoMap')")


# --------------------------------------------------------------------------- #
# the actual map improvement
# --------------------------------------------------------------------------- #
# Verified against the live dashboard: the geoMap widget carries no center/zoom;
# the map auto-fits to each layer's metadata.extent [minLon, minLat, maxLon,
# maxLat]. The excavator layer's extent was the default [-10,-10,10,10] around
# null-island [0,0], so the map opened in the Gulf of Guinea. Setting the extent
# to a small box around the real site near Muscat centres + zooms the map there.
def improve_map(settings, *, lat, lon, extent_deg):
    """Patch each layer's extent to a small box around (lat, lon). Returns the
    list of changes made so APPLY can report exactly what it did."""
    changes = []
    layers = _main(settings).get("layers", []) or []
    if not layers:
        changes.append("!! settings has no layers[] -- cannot set the map view")
        return changes
    bbox = [round(lon - extent_deg, 6), round(lat - extent_deg, 6),
            round(lon + extent_deg, 6), round(lat + extent_deg, 6)]
    for lyr in layers:
        meta = lyr.setdefault("metadata", {})
        old = meta.get("extent")
        meta["extent"] = bbox
        changes.append(f"layer {lyr.get('name')!r}: extent {old} -> {bbox}")
    return changes


def relocate_feature(client, vector_uuid, lat, lon):
    """Move the single status feature to (lat, lon) so the marker sits at the
    site immediately (the Pi keeps it there / updates it with live GNSS). Returns
    a status string; never raises out (best-effort)."""
    try:
        layer = client.get_vector(vector_uuid)
        feats = layer.get_features(limit=1, out_srid=4326)
        if not feats:
            return "no feature found to move (Pi will create one on first run)"
        feat = feats[0]
        geom = feat.data.setdefault("geometry", {"type": "Point", "coordinates": [0, 0]})
        old = list(geom.get("coordinates", []))
        geom["coordinates"] = [lon, lat]
        feat.save()
        return f"feature {getattr(feat, 'id', '?')} moved {old} -> [{lon}, {lat}]"
    except Exception as e:
        return f"feature relocate skipped ({e})"


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="Improve the GEOMind excavator dashboard map")
    ap.add_argument("--host", default="https://app.geo-mind.ai")
    ap.add_argument("--user", default="pdo.excavator")
    ap.add_argument("--pass", dest="password", default="PDO@excavator1")
    ap.add_argument("--dashboard-uuid", default="dcf356e7-ae85-4ea4-8430-843c11f75d9c")
    ap.add_argument("--dashboard-name", default="excavator1_dashboard")
    ap.add_argument("--home-lat", type=float, default=23.5900, help="site latitude (Muscat)")
    ap.add_argument("--home-lon", type=float, default=58.4059, help="site longitude (Muscat)")
    ap.add_argument("--extent-deg", type=float, default=0.02,
                    help="half-width (degrees) of the map view box around the site "
                         "(0.02 deg ~ 2 km -> street-level zoom)")
    ap.add_argument("--vector-uuid", default="c44b2bc5-028f-42dd-b340-8eca36560b17",
                    help="status vector uuid (used to move the feature to the site)")
    ap.add_argument("--no-move-feature", action="store_true",
                    help="with --apply, do NOT move the live feature (only edit the view)")
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "dashboard_settings.json"),
        help="where to dump the downloaded settings")
    ap.add_argument("--apply", action="store_true",
                    help="actually patch + save the dashboard (default: read-only inspect)")
    ap.add_argument("--insecure", action="store_true", help="skip TLS verification")
    args = ap.parse_args()

    print(f"connecting to {args.host} as {args.user} ...")
    try:
        client = GeoboxClient(host=args.host, username=args.user,
                              password=args.password, verify=not args.insecure)
    except Exception as e:
        print("LOGIN FAILED:", e)
        print("-> fix the credentials; do NOT retry in a loop (lockout risk).")
        sys.exit(1)

    # fetch the dashboard (by uuid, falling back to name)
    dash = None
    try:
        dash = client.get_dashboard(uuid=args.dashboard_uuid)
    except Exception as e:
        print(f"get_dashboard(uuid) failed ({e}); trying by name ...")
    if dash is None:
        dash = client.get_dashboard_by_name(name=args.dashboard_name)
    if dash is None:
        print("dashboard not found (check --dashboard-uuid / --dashboard-name)")
        sys.exit(1)

    settings = dash.data.get("settings") or {}
    print(f"dashboard '{dash.data.get('display_name') or dash.data.get('name')}' "
          f"loaded (uuid {dash.uuid})")

    # always dump what we downloaded so it can be reviewed / diffed
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(settings, f, indent=2, ensure_ascii=False)
    print(f"settings written -> {args.out}")
    print("structure:")
    summarize(settings)

    if not args.apply:
        print("\nINSPECT ONLY (no changes made). Review the dump above, then "
              "re-run with --apply to improve + save the map.")
        return

    # back up before touching the live dashboard
    bak = os.path.splitext(args.out)[0] + ".bak.json"
    with open(bak, "w", encoding="utf-8") as f:
        json.dump(settings, f, indent=2, ensure_ascii=False)
    print(f"\nbackup of current settings -> {bak}")

    # 1) move the live feature to the site so the marker is correct right away
    if not args.no_move_feature:
        # prefer the vector uuid embedded in the layer entry, else the CLI default
        vuuid = args.vector_uuid
        layers = _main(settings).get("layers", []) or []
        if layers:
            vuuid = (layers[0].get("metadata", {}) or {}).get("uuid") or vuuid
        print("feature:", relocate_feature(client, vuuid, args.home_lat, args.home_lon))

    # 2) patch the map view (layer extent) to the site
    changes = improve_map(settings, lat=args.home_lat, lon=args.home_lon,
                          extent_deg=args.extent_deg)
    print("map view changes:")
    for c in changes:
        print(f"  * {c}")

    try:
        dash.update(settings=settings)
        print("\nSAVED. Open the dashboard to confirm the map opens on the site "
              "near Muscat with the equipment marker visible.")
    except Exception as e:
        print(f"\nSAVE FAILED: {e}")
        print(f"-> your data is safe; the previous settings are in {bak}")
        sys.exit(1)


if __name__ == "__main__":
    main()
