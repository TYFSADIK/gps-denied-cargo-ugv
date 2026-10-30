"""Dynamic Radius Outlier Removal (DROR), Charron, Phillips & Waslander, CRV 2018.

Falling snow shows up as sparse points near the sensor. A fixed-radius outlier
filter either keeps snow up close or deletes real, naturally sparse points far
away. DROR scales the search radius with range:

    r_search = beta * range * alpha     (alpha = horizontal angular resolution, rad)

and drops points with fewer than k_min neighbours inside r_search.
"""
import numpy as np
from scipy.spatial import cKDTree


def dror(points: np.ndarray, alpha_rad=np.deg2rad(0.2), beta=3.0, k_min=3, r_min=0.04):
    """points (N,3+) in sensor frame. Returns boolean mask of points to KEEP."""
    xyz = points[:, :3]
    rng = np.linalg.norm(xyz[:, :2], axis=1)
    radius = np.maximum(beta * rng * alpha_rad, r_min)
    tree = cKDTree(xyz)
    counts = tree.query_ball_point(xyz, radius, return_length=True)   # vectorised, per-point radius
    return counts - 1 >= k_min


def synthetic_scan(n_snow=600, rings=16, alpha_rad=np.deg2rad(0.2), rng=None):
    """A wall 10 m away scanned like a 16-ring spinning LiDAR, plus snow flakes within 3 m.

    Returns (points, is_snow).
    """
    rng = rng or np.random.default_rng(0)
    az = np.arange(-np.pi / 2, np.pi / 2, alpha_rad)
    elev = np.deg2rad(np.linspace(-15, 15, rings))
    A, E = np.meshgrid(az, elev)
    keep = np.abs(10 * np.tan(E)) < 2.0          # wall is 4 m tall
    A, E = A[keep], E[keep]
    wall = np.column_stack([10 * np.cos(A), 10 * np.sin(A), 10 * np.tan(E)]) + rng.normal(0, 0.01, (A.size, 3))
    r = rng.uniform(0.5, 3.0, n_snow)
    a = rng.uniform(-np.pi, np.pi, n_snow)
    snow = np.column_stack([r * np.cos(a), r * np.sin(a), rng.uniform(-0.5, 2.0, n_snow)])
    return np.vstack([wall, snow]), np.r_[np.zeros(len(wall), bool), np.ones(n_snow, bool)]
