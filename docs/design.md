# CitrusSwerve: Design Doc

3-module coaxial swerve drivetrain. Small scale (N20 gearmotors), one ESP32 on the robot, ROS 2 on a laptop.
Status: **draft v0.3**. Values marked *(TBD)* are placeholders to be set in CAD or testing. Items marked *(verify)* come from memory and must be checked against datasheets or catalogs.

## 1. Architecture

```
 Laptop: ROS 2 (micro-ROS agent, slam_toolbox, teleop, rviz)
        ^
        | WiFi (micro-ROS over UDP)
        v
 ESP32 DevKit V1 (on robot)
   - swerve kinematics, odometry
   - 3x drive velocity loops, 3x steer position loops
   - AS5600 reads, navX reads, lidar UART passthrough
   - watchdog: motors stop if no cmd_vel for ~200 ms
        |
   3x module: [steer N20, no encoder] [drive N20 + encoder] [AS5600] [DRV8833 shared]
   + navX2-MXP (yaw) + 2D lidar (LD06/LD19 class)
```

- Closed loops run on one ESP32 core at fixed rates. WiFi and micro-ROS run on the other, so network jitter cannot disturb the loops.
- Kinematics run on the ESP32, so driving does not depend on the laptop being responsive. The laptop only sends `cmd_vel` and consumes odometry, IMU and scan data.
- All heavy processing (SLAM) is on the laptop.

## 2. Module mechanical concept (coaxial, nested shafts)

Both motors sit on the fixed chassis, so no wires cross the rotating joint and steering can rotate continuously. There are four moving bodies: the **sleeve**, the **inner shaft**, the **wheel axle**, and the **fork** (which rotates with the sleeve).

- **Steer path:** steer N20 -> metal pinion -> steer gear on a hollow outer sleeve -> rotates the module (fork + wheel).
- **Drive path:** drive N20 -> metal pinion -> 1:1 gear -> inner shaft (concentric inside the sleeve) -> bevel gear A -> bevel gear B on the wheel axle.
- **Steering sensor:** AS5600 reads a diametric magnet on a small 1:1 gear meshed with the steer gear. An equal-tooth gear keeps the reading absolute over 360°. The sensor is fixed to the chassis. *(Alternative: radially magnetized ring magnet on the hollow shaft.)*
- **Wheel contact on the steering axis.** The contact patch must sit on (or very near) the steering axis. If it is offset, the contact point travels in a circle as the module steers and the robot moves even with the wheel undriven. Check this in CAD.
- Gear geometry comes from the Onshape gear FeatureScript. *(TBD: module, tooth counts, center distances.)*

### 2.1 Bearings and bushings

Plan: **hybrid.** Real bearings where load and steering smoothness matter, printed bushings where space matters.

| Location | Plan | Size *(verify against catalog)* |
|---|---|---|
| Sleeve in the chassis plate (x2, spaced apart) | Ball bearings. They carry robot weight, steer gear side load, and tipping moments, and they sit inside the steering loop. | 6701, 12×18×4 mm |
| Inner shaft inside the sleeve | Printed bushings on a steel rod to start. Switch to bearings if draggy or loose. | 3 mm rod. Bearing option: MR83, 3×8×3 mm |
| Sleeve geometry | 8 mm bore, about 12 mm outside diameter (2 mm wall) | |
| Slimmer variant | 2 mm shaft, MR62 (2×6×2.5), sleeve 6 mm bore and 10 mm OD on 6700 (10×15×4). Shaft is more fragile. | |

Notes on the original sizes: an 8 mm sleeve bore running on 8×16 mm bearings is **not** a consistent stack (bore and outer diameter can't both be 8 mm). The table above replaces it.

Printed bushing rules:
- Bear on a steel rod, never a printed shaft in a printed bore.
- Design 0.1–0.2 mm clearance and ream the bore by hand. Print the bore axis vertically.
- PETG or nylon over PLA. Use a drop of light grease or PTFE lube.
- Test the nested stack on its own before building the full module. Misalignment between bushing pairs causes binding.

### 2.2 Thrust and mesh adjustment

- Bevel gears push axially on both shafts. Add a thrust washer (nylon or PTFE) under the bevel gear and a shaft collar or shoulder to stop the inner shaft sliding.
- Bevel gears only mesh well at the right mounting distance. Give the bevel mounts shims or slots.
- Prefer **straight miter gears (1:1)**: equal tooth counts balance the thrust, and mounting is simplest.
- Bevel efficiency is a few percent below spur gears (ballpark 93–98% per mesh against 97–99%). Alignment and print quality matter more than gear type at this scale.

### 2.3 Attaching gears to N20 shafts

- **Motor-side pinions:** buy metal pinions (N20 pinion, 3 mm D bore, match the module to the mating gear). Printed teeth at module 0.5 are fragile.
- **Larger printed gears on the inner shaft:** printed hub with an **M2 set screw threaded into a heat-set insert**. The screw bears on the D flat.
  - Heat-set inserts survive repeated tightening, unlike threads tapped into plastic. Follow the insert supplier's hole size and depth. Print the hole axis vertically where possible, and keep at least about 1 mm of plastic around the insert. The insert sits radially in the hub wall, so expect a hub OD of roughly 9–10 mm *(check against your insert)*.
  - N20 shafts are short (around 9–10 mm, check your motor), so keep the hub short.
- Printed gears: use module 0.8–1.0. Module 0.5 is too fine to print reliably.

## 3. Steer-drive coupling

With the inner shaft held still and the module rotating, bevel gear B walks around bevel gear A and the wheel rolls. Wheel rotation depends on the drive shaft angle *relative to the module*:

```
ω_wheel = r · (ω_drive − ω_steer)
```

`r` is the total ratio from relative inner-shaft rotation to wheel rotation (1.0 with a 1:1 bevel pair). The sign depends on the bevel arrangement. **Verify in Onshape (gear relation mates) and on the bench.**

To get a desired wheel speed while steering:

```
ω_drive_cmd = ω_wheel_cmd / r + ω_steer
```

- `ω_steer` is the module rate (steer motor rate / steer ratio), from the filtered AS5600 derivative, plus the commanded steer rate as feedforward.
- Odometry must use `r(ω_drive − ω_steer)`, not the drive encoder alone.
- **Stopped wheel, steering:** with `ω_wheel = 0` the drive shaft must still turn at `ω_steer`. The axle and bevel B pivot about the steering axis, and the wheel must not roll relative to the module, so gear A has to co-rotate with the module. The compensation is needed whenever the module steers, not only when driving. If the motor does not compensate, the wheel is forced to roll and the robot creeps. The N20 gearbox resists back-driving, so an unpowered drive motor acts as a locked shaft. **Simulation test case:** steering with zero commanded speed must produce zero wheel rotation.
- **Headroom:** the drive motor must also cover the steer rate. If `ω_drive_cmd` exceeds the motor's limit, reduce wheel speed and keep steering priority.

### 3.1 Where to put drive reduction

A reduction `G` between the drive motor and the inner shaft multiplies the steer term at the motor:
`ω_motor = G · (ω_wheel/r + ω_steer)`.
A reduction placed **at the bevel or wheel end** (after the coupling point, `r < 1`) does not multiply it:

| 2:1 reduction placed | Motor speed needed |
|---|---|
| Before the shaft (G = 2) | 2·ω_wheel + 2·ω_steer |
| At the bevel (r = 0.5) | 2·ω_wheel + 1·ω_steer |

So keep motor-to-shaft at 1:1 and put any extra drive reduction in the bevel stage, if the fork can fit the larger gear.

## 4. Kinematics (3 modules)

Module positions (robot frame), radius `R`, angles φ = 90°, 210°, 330° *(TBD from CAD)*:

```
p_i = R · (cos φ_i, sin φ_i)
v_i = (vx − ω·y_i,  vy + ω·x_i)
speed_i = |v_i|,  angle_i = atan2(v_i.y, v_i.x)
```

- **Angle optimization:** if the steer change exceeds 90°, rotate the target 180° and reverse the wheel speed.
- **Desaturation:** scale all wheel speeds together if any exceeds the limit.
- **Cosine scaling:** scale wheel speed by cos(steer error) to reduce scrub while turning.
- **Zero command:** hold the last angle.
- Keep the chassis mounting flexible enough that a 4th module position could be added later.

## 5. Motors, ratios and targets

Drive and steer use the same part, a **500 rpm, 6 V N20**. The drive motors have the encoder version. This gives one SKU and shared spares. 500 rpm is a good general-purpose speed.

**Ratios (current plan, TBD)**

| Path | Ratio | Result (30 mm wheel, 70% of no-load when loaded) |
|---|---|---|
| Drive: motor to shaft | 1:1 | |
| Drive: bevel | 1:1, up to 1.5:1 if the module is draggy | 1:1 -> 500 rpm, 0.79 m/s no load, ~0.55 m/s loaded. 1.5:1 -> 333 rpm, ~0.52 / ~0.37 m/s |
| Steer: motor to module | 4:1 to 5:1 | 4:1 -> 125 rpm no load, 360° in ~0.7 s loaded. 5:1 -> 100 rpm, ~0.86 s |
| Sensor gear to steer gear | 1:1 | Keeps the AS5600 absolute over 360° |

- Steer takes more reduction than drive: 0.7–0.9 s per full turn is fine, it calms the position loop, and it reduces the coupling cost.
- If more torque is needed, prefer a slower motor (for example 300 rpm), which puts the reduction inside the motor's metal gearbox, over adding printed stages.

**Torque reality check (rough estimates, verify on the datasheet).** 1 kg over 3 wheels at 1 m/s² needs about 0.33 N per wheel, about 5 mN·m at a 30 mm wheel. Floor scrub when steering in place is of a similar size. A 500 rpm N20 should have far more stall torque than this, so the real losses are **friction** in the bushings, printed gears and bevel mesh. Do not plan on heavy reduction to compensate for a draggy build.

| Item | Target |
|---|---|
| Robot mass | ≤ 1 kg |
| Wheel diameter | ~30 mm *(TBD)* |
| Usable speed while steering | ~0.45 m/s |
| Steer accuracy | ≤ 1° (backlash dominated) |
| Steer loop | ≥ 500 Hz |
| Drive loop | ≥ 200 Hz |

## 6. Electronics and parts (bill of materials)

| Part | Qty | Notes |
|---|---|---|
| ESP32 DevKit V1 (classic) | 1 | See `electrical/pinmap.md` |
| DRV8833 | 3 | One per module (steer + drive). 10.8 V max, 1.5 A continuous per channel |
| N20 500 rpm 6 V, with encoder | 3 | Drive |
| N20 500 rpm 6 V, no encoder | 3 | Steer |
| AS5600 + diametric magnet | 3 | Fixed address 0x36 |
| TCA9548A I²C mux | 1 | Owned. Address 0x70 |
| navX2-MXP | 1 | Yaw. Level shifting *(verify I/O is 3.3 V)* |
| 2D lidar (LD06 / LD19 class) | 1 | 5 V power, UART out at 3.3 V *(verify)* |
| 5 V 3 A BEC | 1 | Electronics only. Confirm input range covers 2S (8.4 V) |
| 2S LiPo, 800 mAh | 1 | Min 6.4 V cutoff, check C rating |
| Metal N20 pinions | 6 | 3 mm D bore, module to match mating gear |
| Bevel / miter gears, 1:1 | 3 pairs | Source or print. Add shimming |
| Ball bearings | 6 (6701) | 2 per module for the sleeve. 6× MR83 optional for inner shafts |
| Steel rod, 3 mm | 3 | Inner shafts |
| Thrust washers, shaft collars | 3 sets | Nylon or PTFE washers |
| M2 heat-set inserts and M2 set screws | per hub | Inserts for set screws and for bolting gears to hubs |
| Small parts | | Power switch, ~5 A fuse, battery connector, Schottky diode on BEC output, battery sense divider, 470–1000 µF bulk caps near the drivers, 0.1 µF ceramic across each motor |

## 7. Power

```
2S LiPo (6.4–8.4 V) --+-- fuse/switch --+-- DRV8833 VM (motors, battery-direct)
                                        +-- BEC -> 5 V -> ESP32 VIN, lidar, navX
                                                    ESP32 3V3 -> AS5600, mux, encoders
```

- Motors are 6 V parts on a 7.4–8.4 V pack. The firmware caps PWM duty at `6 V / V_battery` (about 71% at 8.4 V), using the measured pack voltage. No separate buck regulator is needed. If speed varies too much as the pack drains, add a 6 V buck later.
- The battery sense divider doubles as a low-voltage cutoff (stop motors below ~6.4 V).
- Estimated 5 V rail: ESP32 0.25–0.5 A, lidar 0.3–0.4 A, navX ~0.1 A, sensors ~0.05 A. Total about 1–1.3 A of the 3 A available.
- Tie motor and logic grounds at one point.
- All ESP32 pins are 3.3 V only and not 5 V tolerant. Power every sensor and encoder from 3.3 V.

## 8. Sensors and bus layout

- **AS5600 x3:** one I²C bus through the TCA9548A, one AS5600 per channel. Sample at ≥ 500 Hz, filter the derivative. Estimated ~0.8 ms per cycle for three reads at 400 kHz *(confirm on bench)*.
- **navX2:** I²C address 0x32 *(verify)* does not collide with 0x36 or 0x70, so it sits on the main bus next to the mux. Read it at 50–100 Hz. There is no ready-made ESP32 library as far as I know, so expect to read its registers yourself.
- **Lidar:** UART receive only. About 4.5k points/s, around 450 points per scan at 10 Hz. Either parse on the ESP32 and publish `LaserScan` through micro-ROS (needs the message buffers enlarged), or forward the raw serial stream to the laptop and parse there.
- **Mux pull-ups:** check for pull-ups on the upstream and each downstream channel. Do not stack many parallel pull-ups.

## 9. Decisions log

- **3 modules, not 4.** A 4th module needs 28 signals, which doesn't fit a classic ESP32, plus more parts and weight. Chassis keeps room for a 4th. Revisit after one module works.
- **ESP32 only, no Pi or Pico.** Saves the Pi's power draw on an 800 mAh pack. Arduino Nano rejected (pins, interrupts, RAM).
- **DRV8833 over TB6612.** Smaller, 2 pins per channel, lower on-resistance. Needs bulk capacitance for voltage spikes (10.8 V max).
- **Battery-direct motor power with a duty cap** instead of a 6 V buck.
- **Lidar chosen over camera/AprilTag.** Processing is on the laptop either way.
- **Ring-gear (internal+external teeth) drive rejected.** It would need very large bearings or a roller ring, and it keeps the same coupling term anyway.

## 10. Open questions

1. Gear tooth counts and center distances (FeatureScript).
2. Bevel arrangement and the sign of the coupling term.
3. Stall current and torque of the chosen N20 against the DRV8833 limit.
4. Whether lidar is parsed on the ESP32 or on the laptop.
5. Whether the fork can fit a 1.5:1 bevel pair, or whether drive stays at 1:1.
6. Inner-shaft support: printed bushings or MR83 bearings.

## 11. Milestones

1. Onshape model of the four moving bodies with revolute and gear relation mates. Confirm the coupling direction.
2. Bearing stack test piece: sleeve, inner shaft and their supports only. Spin by hand.
3. Python kinematics + simulation, including the coupling term and the stopped-wheel steer case.
4. Single-module bench rig: steer loop and drive loop.
5. Three modules + ESP32 firmware + micro-ROS link.
6. navX, odometry, teleop.
7. Lidar + `slam_toolbox` on the laptop.
