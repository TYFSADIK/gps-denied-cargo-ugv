"""Run: python demo.py  -> snow filtering, radar ego-velocity with a moving target, slip, energy vs spec."""
import numpy as np

from ugv import energy
from ugv.radar_ego import ego_velocity
from ugv.slip import SlipEstimator
from ugv.snow_filter import dror, synthetic_scan

pts, is_snow = synthetic_scan()
keep = dror(pts)
print(f"DROR: removed {np.mean(~keep[is_snow]):.0%} of snow, kept {np.mean(keep[~is_snow]):.0%} of wall")

rng = np.random.default_rng(1)
theta = rng.uniform(-1.0, 1.0, 80)
v_true = np.array([2.0, 0.1])
vr = -(np.cos(theta) * v_true[0] + np.sin(theta) * v_true[1]) + rng.normal(0, 0.03, 80)
vr[:10] += 4.0                                  # a vehicle driving past
v, inl, sig = ego_velocity(theta, vr)
print(f"radar ego-velocity {v.round(3)} (true {v_true}), {inl.sum()}/80 inliers, sigma {sig.round(3)}")

se = SlipEstimator()
for vw, vg in [(1.0, 0.95)] * 5 + [(1.5, 0.4)] * 10:
    se.update(vw, vg)
print(f"slip={se.slip:.2f} spinning={se.spinning} wheel sigma={se.wheel_variance(1.5) ** 0.5:.2f} m/s")

runs = [{"payload_kg": p, "temp_c": t, "surface": s, "wh_per_km": 60 + 0.35 * p - 1.2 * min(t, 20) + o + rng.normal(0, 3)}
        for p in (0, 10, 20, 30) for t in (10, -5, -20) for s, o in (("paved", 0), ("packed", 25), ("loose", 70))]
beta, r2 = energy.fit(runs)
print(f"energy model R^2={r2:.3f}")
for row in energy.spec_check(beta, battery_wh=2000):
    print("  ", row)
