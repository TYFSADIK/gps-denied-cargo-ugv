"""Payload / temperature / surface energy model fitted from winter runs, checked against the spec.

Run log rows: {payload_kg, temp_c, surface ('packed'|'loose'|'gravel'|'paved'), speed_kmh, wh_per_km}
Model: wh_per_km = b0 + b1*payload + b2*min(temp,20) + surface offsets  (least squares)
"""
import numpy as np

SURFACES = ["paved", "gravel", "packed", "loose"]


def _design(rows):
    X = []
    for r in rows:
        x = [1.0, r["payload_kg"], min(r["temp_c"], 20.0)]
        x += [1.0 if r["surface"] == s else 0.0 for s in SURFACES[1:]]
        X.append(x)
    return np.array(X)


def fit(rows):
    X = _design(rows)
    y = np.array([r["wh_per_km"] for r in rows])
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ beta
    r2 = 1 - np.sum((y - pred) ** 2) / np.sum((y - y.mean()) ** 2)
    return beta, float(r2)


def predict(beta, payload_kg, temp_c, surface):
    return float(_design([{"payload_kg": payload_kg, "temp_c": temp_c, "surface": surface, "wh_per_km": 0}])[0] @ beta)


def endurance_h(battery_wh, wh_per_km, speed_kmh, usable=0.8, hotel_w=40.0):
    """Hours of continuous motion; hotel_w = compute + sensors + radio."""
    p = wh_per_km * speed_kmh + hotel_w
    return battery_wh * usable / p


def stationary_h(battery_wh, payload_w, idle_w=15.0, heater_w=0.0, usable=0.8):
    return battery_wh * usable / (payload_w + idle_w + heater_w)


def spec_check(beta, battery_wh):
    """MINERVA lines: 4 h at 15 km/h unladen; 7 km/h cross-country with 140 kg."""
    rows = []
    for name, payload, speed, surf, need_h in [("4 h @ 15 km/h unladen, paved, -20 C", 0, 15, "paved", 4.0),
                                               ("cross-country 140 kg @ 7 km/h, loose snow, -20 C", 140, 7, "loose", 4.0)]:
        h = endurance_h(battery_wh, predict(beta, payload, -20, surf), speed)
        rows.append({"requirement": name, "achieved_h": round(h, 2), "needed_h": need_h,
                     "battery_needed_wh": round(battery_wh * need_h / h)})
    return rows
