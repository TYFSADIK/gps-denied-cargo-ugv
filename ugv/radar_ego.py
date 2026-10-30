"""Ego-velocity from a single mmWave radar scan (Doppler), robust to moving targets.

For a static point at azimuth theta, the measured radial velocity is
    v_r = -(vx cos(theta) + vy sin(theta))
so one scan of static clutter over-determines (vx, vy). RANSAC rejects moving
objects. Unlike LiDAR, this keeps working in falling snow, which is why the EKF
takes radar velocity rather than asking a small radar to do SLAM.
"""
import numpy as np


def _lsq(theta, vr):
    A = -np.column_stack([np.cos(theta), np.sin(theta)])
    v, *_ = np.linalg.lstsq(A, vr, rcond=None)
    return v, A


def ego_velocity(theta, vr, iters=100, thresh=0.15, rng=None):
    """theta, vr: arrays per radar point. Returns (v_xy, inlier_mask, sigma_xy)."""
    rng = rng or np.random.default_rng(0)
    n = len(theta)
    if n < 3:
        return None, np.zeros(n, bool), None
    best = np.zeros(n, bool)
    for _ in range(iters):
        idx = rng.choice(n, 2, replace=False)
        if abs(np.sin(theta[idx[0]] - theta[idx[1]])) < 0.05:
            continue
        v, _ = _lsq(theta[idx], vr[idx])
        res = np.abs(vr + np.cos(theta) * v[0] + np.sin(theta) * v[1])
        inl = res < thresh
        if inl.sum() > best.sum():
            best = inl
    v, A = _lsq(theta[best], vr[best])
    resid = vr[best] - A @ v
    s2 = resid @ resid / max(best.sum() - 2, 1)
    cov = s2 * np.linalg.inv(A.T @ A)
    return v, best, np.sqrt(np.diag(cov))
