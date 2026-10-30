# GPS-Denied Cargo UGV for Snow (build guide)

Private research repo. A GPS-denied cargo ground vehicle for snow: LiDAR-inertial odometry, EKF slip handling, payload and energy sweeps, and honest spec-gap tables against the MINERVA ground challenge lines.

Full write-up and phase plan: [`docs/project-page.md`](docs/project-page.md)

## What's here

| File | Purpose |
|---|---|
| `ugv/snow_filter.py` | **DROR** (Charron et al. 2018): range-scaled radius outlier removal for falling snow, run before odometry |
| `ugv/radar_ego.py` | mmWave **Doppler ego-velocity** with RANSAC (rejects moving vehicles). Keeps working when LiDAR degrades in snow |
| `ugv/slip.py` | Slip ratio from wheel vs ground velocity, and wheel-odometry variance that grows with slip |
| `ugv/energy.py` | Wh/km model (payload, temperature, surface: paved, gravel, packed, loose), endurance and stationary hours, spec check |
| `config/kiss_icp_snow.yaml` | KISS-ICP settings for snow (min range cuts the near-sensor halo) |

## Quickstart

```bash
pip install -r requirements.txt
python demo.py && python -m unittest -v
```

## Phase checklist
- [ ] Phase 0: Gazebo/Isaac snow-field sim, with DROR → KISS-ICP → EKF in the loop
- [ ] Phase 1: bench bring-up (LiDAR, IMU, mmWave radar), time sync with PTP or chrony
- [ ] Phase 3 winter runs: log `surface` (packed or loose) every run, since the research shows it dominates energy use
- [ ] Phase 4 payload sweep 0/10/20/30 kg, then fit `energy.fit` and publish the spec-gap table from `energy.spec_check`
- [ ] Teach-and-repeat with radar as the fallback localiser near snowbanks
