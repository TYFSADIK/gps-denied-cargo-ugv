import unittest

import numpy as np

from ugv import energy
from ugv.radar_ego import ego_velocity
from ugv.slip import SlipEstimator
from ugv.snow_filter import dror, synthetic_scan


class TestUGV(unittest.TestCase):
    def test_dror_removes_snow_keeps_wall(self):
        pts, snow = synthetic_scan()
        keep = dror(pts)
        self.assertGreater(np.mean(~keep[snow]), 0.8)
        self.assertGreater(np.mean(keep[~snow]), 0.9)

    def test_radar_ego_rejects_mover(self):
        rng = np.random.default_rng(2)
        th = rng.uniform(-1, 1, 60)
        vr = -(np.cos(th) * 1.5) + rng.normal(0, 0.02, 60)
        vr[:8] -= 3
        v, inl, _ = ego_velocity(th, vr)
        self.assertAlmostEqual(v[0], 1.5, delta=0.05)
        self.assertFalse(inl[:8].any())

    def test_slip(self):
        s = SlipEstimator(alpha=1.0)
        self.assertAlmostEqual(s.update(1.0, 0.5), 0.5)
        self.assertTrue(s.spinning)

    def test_energy_fit(self):
        rows = [{"payload_kg": p, "temp_c": t, "surface": "paved", "wh_per_km": 50 + 0.5 * p - t}
                for p in (0, 20, 40) for t in (0, -10, -20)]
        beta, r2 = energy.fit(rows)
        self.assertGreater(r2, 0.99)
        self.assertAlmostEqual(energy.predict(beta, 10, -5, "paved"), 60, delta=0.5)


if __name__ == "__main__":
    unittest.main()
