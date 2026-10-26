# Mini-ROV: Live Underwater Image Delivery (Design + MATLAB Simulation)

A low-cost, fully 3D-printed mini ROV for shallow water, and a MATLAB simulator used to check its energy budget, controllers and mission logic before building.

**Core idea:** Wi-Fi and 4G do not work underwater, so the antenna stays above the surface. Video travels from the vehicle over an Ethernet tether to a floating buoy, where a 4G modem relays it to a cloud server and a web page.

> Status: design and simulation stage. All vehicle and power parameters are assumptions, not measurements. The 4G link is not simulated yet.

## System overview

```
Camera -> Raspberry Pi (detector, prioritisation) -> Ethernet tether
       -> surface buoy (4G modem) -> cloud server (WebRTC / SRT) -> website
```

- **Vehicle:** 3D-printed frame (PETG / ASA), sealed transparent tube with dome, 2 horizontal thrusters + 1 vertical thruster, Raspberry Pi with Camera Module 3 Wide, white LEDs, 4S LiPo battery.
- **Surface relay:** buoy with 4G modem; antenna above water.
- **Server:** small cloud server (for example MediaMTX) that serves the stream to a web page.
- **Data prioritisation:** frames with detections are sent first; others are sent at reduced quality when bandwidth is low.

## Simulator

`rov_sim.m` is a single-file MATLAB script (base MATLAB, R2016b or newer, no toolboxes).

### Run

```matlab
cd('path/to/this/folder')
rov_sim            % run and play
rov_sim('video')   % also record rov_sim.mp4 (first run only)
```

### What you can change (Settings panel)

| Setting | Options |
| --- | --- |
| Environment | Shallow (seabed 6 m, operating depth 3 m) / Medium (seabed 15 m, operating depth 8 m) |
| Water quality | Clear 8 m / Moderate 5 m / Turbid 3 m / Very turbid 1.5 m visibility |
| Power mode | Eco 0.4 m/s / Normal 0.6 m/s / Performance 0.9 m/s |
| 4G uplink | Off / on at the surface only / always on |
| Controller gains | Depth Kp, Ki, Kd; Heading Kp, Kd; Speed gain |
| Battery | Capacity (mAh), voltage (V), usable fraction (%) |

Press **Apply and re-run** after changing values. **Reset defaults** restores the baseline.

### Views and outputs

- 3D chase view of the vehicle (Chase / Side / Front / Top), mission map, simulated onboard camera with detection overlay.
- Analysis plot: depth, thrusters, stacked power by component, battery endurance.
- Results panel: energy, average and peak load, runtime, energy shares, depth and heading errors.
- **Save report** writes a folder `rov_report_<timestamp>` with `report.html`, plots, a screenshot and `log.csv`. Open `report.html` in a browser and use Print > Save as PDF if you need a PDF.

### Mission phases

1. Dive
2. Search along waypoints
3. AI tracking of the target
4. Return to base
5. Ascend
6. Surfaced (4G uplink)

Below 20% battery the vehicle returns early. At 0% it stops the thrusters and floats up.

## Model summary

- 4-DOF dynamics: surge, heave, yaw (pitch and roll are visual only), first-order thruster lag of 0.15 s, time step 0.05 s.
- Depth PID with rate-limited reference and anti-windup; heading PD; speed feed-forward plus proportional term; differential thrust mixing.
- Detection: target must be inside the camera field of view (45 degrees horizontal, 35 degrees vertical half-angle) and within the water visibility range.
- Power: `P_bat = (P_thr + P_Pi + P_AI + P_light + P_4G + P_ESC) / eta`, with `P_thr = P_max * sum((|T_i| / T_max)^1.5)`, `eta = 0.90`.

## Baseline result

Shallow water, clear, normal power, 4G at surface only, 5000 mAh at 14.8 V (62.9 Wh usable):

| Quantity | Value |
| --- | --- |
| Mission duration | 72 s |
| Energy per mission | 0.92 Wh |
| Average / peak load | 46 W / 158 W |
| Estimated runtime | about 82 min (about 66 min to the 20% reserve) |
| Energy share (thrusters / Pi + AI / lights / 4G / other) | 52% / 19% / 16% / 1% / 13% |
| Depth overshoot / RMS error | 0.03 m / 0.24 m |
| Heading error while tracking (RMS / max) | 4.6 deg / 6.8 deg |

## Limitations

- Mass, drag, thrust and power values are assumed; replace them with bench measurements.
- Detection is a geometric test, not a trained model.
- The tether is drawn but its drag is not modelled; waves, currents and battery voltage sag are not modelled.
- The 4G link (bandwidth, latency, packet loss) is not simulated.
- Runtime assumes the mission repeats back-to-back.

## Roadmap

- [ ] Measure real 4G throughput and latency at the test site
- [ ] Adaptive bitrate streaming and priority-based frame queue
- [ ] Bench-test thrusters and replace assumed power values
- [ ] Trained nano object detector on the Raspberry Pi
- [ ] Pool and shallow-sea trials, compare with the simulator

## Repository layout

```
rov_sim.m    MATLAB simulator (single file)
README.md    this file
```

## License

Add a license of your choice (for example MIT) before publishing.
