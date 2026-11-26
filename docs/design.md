# CitrusSwerve: Design Doc

3-module coaxial swerve drivetrain. Small scale (N20 gearmotors), one ESP32 on the robot, ROS 2 on a laptop.
Status: **draft v0.5**. Values marked *(TBD)* are placeholders to be set in CAD or testing. Items marked *(verify)* come from memory and must be checked against datasheets or catalogs.

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
   + navX2-MXP (yaw) + 2D lidar (LDROBOT LD06)
```

- Closed loops run on one ESP32 core at fixed rates. WiFi and micro-ROS run on the other, so network jitter cannot disturb the loops.
- Kinematics run on the ESP32, so driving does not depend on the laptop being responsive. The laptop only sends `cmd_vel` and consumes odometry, IMU and scan data.
- All heavy processing (SLAM) is on the laptop.
- **Lidar: LDROBOT LD06**, chosen after weighing several options (section 9). It streams scans over UART by itself, so it needs only one receive pin (GPIO15). Not bought yet, and the drivetrain works without it.

## 2. Module mechanical concept (coaxial, nested shafts)

Both motors sit on the fixed chassis, so no wires cross the rotating joint and steering can rotate continuously. There are four moving bodies: the **sleeve**, the **inner shaft**, the **wheel axle**, and the **fork** (which rotates with the sleeve).

- **Steer path:** steer N20 -> printed pinion -> printed steer gear on a hollow outer sleeve -> rotates the module (fork + wheel).
- **Drive path:** drive N20 -> printed pinion -> 1:1 printed gear -> inner shaft (concentric inside the sleeve) -> bevel gear A -> bevel gear B on the wheel axle.
- **Steering sensor:** AS5600 reads a diametric magnet on a small 1:1 gear meshed with the steer gear. An equal-tooth gear keeps the reading absolute over 360°. The sensor is fixed to the chassis. *(Alternative: radially magnetized ring magnet on the hollow shaft.)*
- **Wheel contact on the steering axis.** The contact patch must sit on (or very near) the steering axis. If it is offset, the contact point travels in a circle as the module steers and the robot moves even with the wheel undriven. Check this in CAD.
- Gear geometry comes from the Onshape gear FeatureScript. *(TBD: module, tooth counts, center distances.)*

### 2.1 Bearings: 625ZZ roller turret

Bearing on hand: **30× 625ZZ** (5×16×5 mm), chosen as a general-purpose size that is reusable in other projects. Everything below uses that one size.

| Location | Plan |
|---|---|
| Sleeve, radial support | **3 radial rollers** (625ZZ on vertical M5 bolts through the chassis plate) at 120°, rolling on the sleeve's outside surface. **One must be adjustable** (slotted mount or eccentric spacer) to set a light, even preload. |
| Sleeve, axial support | **3 thrust rollers** (625ZZ on horizontal bolts) bearing on a flange on the sleeve, plus a retainer lip on the other face so the sleeve cannot lift out. They carry the robot's weight. |
| Inner shaft inside the sleeve | **2× 625ZZ** pressed into the sleeve. Sleeve bore 16 mm, outside diameter about 20 mm (2 mm wall). Shaft is 5 mm. |
| Wheel axle in the fork | 2× 625ZZ *(TBD: axle diameter and size)* |
| Sensor gear axle | Printed bushings, or more 625ZZ |

Notes:
- Footprint: rollers are 16 mm in diameter around a 20 mm sleeve, so the module is roughly 50 mm across.
- **Bearing count:** per module 6 (turret) + 2 (inner shaft) + 2 (wheel axle) = 10, so 30 for three modules. That uses all 30 with nothing spare, and the sensor gear axles need more. Buy a few extra or use bushings there.
- **Rolling surface:** the rollers run on the sleeve, so use a smooth track, such as a sanded printed ring or a thin brass or aluminum tube pressed onto the sleeve. Steel on rough plastic wears flats into it.
- **Binding:** six fixed rollers against one sleeve bind if even one is a fraction of a millimeter out of place. Binding means the parts jam against each other, so motion becomes stiff, notchy, or uneven in some positions. Avoid it by making at least one roller adjustable, keeping each roller axis square, and testing the turret by hand before adding gears.
- **Inner shaft material:** TBD, an M5 bolt or a ground rod. Do not use a printed shaft turning in a printed bore. The inner shaft can run on 625ZZ bearings, or printed bushings on steel as a lighter alternative.

Alternatives considered:
- 688 (8×16×4) sleeve bearings with printed bushings inside, slimmer but uses a different size.
- 6806 (30×42×7) with 608 inside, robust but about 50 mm or more and heavier.
- 6804 (20×32×7) under the sleeve with 625ZZ inside, a good fit but needs a second size, and the roller turret replaced it.

### 2.2 Thrust and mesh adjustment

- Bevel gears push axially on both shafts. Add a thrust washer (nylon or PTFE) under the bevel gear and a shaft collar or shoulder to stop the inner shaft sliding.
- Bevel gears only mesh well at the right mounting distance. Give the bevel mounts shims or slots.
- Prefer **straight miter gears (1:1)**: equal tooth counts balance the thrust, and mounting is simplest. A 5 mm bore makes the miter gears about 14 mm or more in pitch diameter. Printed bevels are fragile at this size *(TBD: print or buy)*.
- Bevel efficiency is a few percent below spur gears (ballpark 93–98% per mesh against 97–99%). Alignment and print quality matter more than gear type at this scale.

### 2.3 Gears and attaching them to shafts

- **All gears are printed, including the motor pinions.** No metal pinions are being bought.
- Use module 0.8–1.0. Module 0.5 is too fine to print reliably.
- Attachment: printed hub with an **M2 set screw threaded into an M2 heat-set insert**. The screw bears on the D flat of the N20 shaft. Heat-set inserts survive repeated tightening, unlike threads tapped into plastic. Follow the insert supplier's hole size and depth. Print the hole axis vertically where possible, and keep at least about 1 mm of plastic around the insert.
- **Pinion boss:** a 10–12T pinion at module 0.8 has a root diameter around 7–8 mm, leaving about 2 mm of wall around a 3 mm D bore. That is too thin for a radial M2 insert. Give the pinion a **boss** (a cylinder about 9–10 mm in diameter) beside the teeth and put the insert in it. The N20 shaft is short (around 9–10 mm, check the motor), so the teeth and boss share that length. Shorter inserts exist, so check your supplier.
- Gears on the 5 mm inner shaft: file a flat on the rod, or use a set screw on the round rod.

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

Drive and steer use the same speed class, a **500 rpm, 6 V N20**. The drive motors are the encoder version (GA12-N20). 500 rpm is a good general-purpose speed.

**Ratios (current plan, TBD)**

| Path | Ratio | Result (30 mm wheel, 70% of no-load when loaded) |
|---|---|---|
| Drive: motor to shaft | 1:1 | |
| Drive: bevel | 1:1, up to 1.5:1 if the module is draggy | 1:1 -> 500 rpm, 0.79 m/s no load, ~0.55 m/s loaded. 1.5:1 -> 333 rpm, ~0.52 / ~0.37 m/s |
| Steer: motor to module | 4:1 to 5:1 | 4:1 -> 125 rpm no load, 360° in ~0.7 s loaded. 5:1 -> 100 rpm, ~0.86 s |
| Sensor gear to steer gear | 1:1 | Keeps the AS5600 absolute over 360° |

- Steer takes more reduction than drive: 0.7–0.9 s per full turn is fine, it calms the position loop, and it reduces the coupling cost.
- **Steer gear size:** a 5:1 reduction with a 10T printed pinion at module 0.8 needs a 50T gear, about 40 mm in pitch diameter, around the sleeve. This sets the module footprint. A two-stage reduction or a smaller ratio would shrink it, so trade it off in the FeatureScript.
- If more torque is needed, prefer a slower motor (for example 300 rpm), which puts the reduction inside the motor's metal gearbox, over adding printed stages.

**Torque reality check (rough estimates, verify on the datasheet).** 1 kg over 3 wheels at 1 m/s² needs about 0.33 N per wheel, about 5 mN·m at a 30 mm wheel. Floor scrub when steering in place is of a similar size. A 500 rpm N20 should have far more stall torque than this, so the real losses are **friction** in the bearings, printed gears and bevel mesh. Do not plan on heavy reduction to compensate for a draggy build.

| Item | Target |
|---|---|
| Robot mass | ≤ 1 kg |
| Wheel diameter | ~30 mm *(TBD)* |
| Usable speed while steering | ~0.45 m/s |
| Steer accuracy | ≤ 1° (backlash dominated) |
| Steer loop | ≥ 500 Hz |
| Drive loop | ≥ 200 Hz |

## 6. Electronics and parts (bill of materials)

"In cart" means it was in the AliExpress cart on 2026-11-25 and **has not been ordered**. "Owned" means it was already on hand.

| Part | Qty | Status / notes |
|---|---|---|
| ESP32 DevKit V1 (classic) | 1 | Owned. See `electrical/pinmap.md` |
| DRV8833 | 3 | In cart. One per module (steer + drive). 10.8 V max, 1.5 A continuous per channel |
| GA12-N20 encoder motor, 500 rpm 6 V | 3 | In cart. Drive. Check encoder supply works at 3.3 V |
| N20 motor, 500 rpm 6 V, no encoder | 3 | In cart. Steer |
| Spare N20 motors | 1 each type | To buy |
| AS5600 board | 3 | In cart. Comes with magnet |
| TCA9548A I²C mux | 1 | In cart. Address 0x70 |
| navX2-MXP | 1 | Owned. Level shifting *(verify I/O is 3.3 V)* |
| 5 V 3 A BEC | 1 | Owned. Confirm input range covers 2S (8.4 V) |
| 2S LiPo, 800 mAh | 1 | Owned. Min 6.4 V cutoff, check C rating |
| 625ZZ bearings | 30 | To buy (30 planned). See section 2.1. A few more recommended |
| M2 heat-set inserts (M2×L3×OD3.2) | 100 | In cart |
| M2 screws, incl. set screws | | Owned |
| M3 heat-set inserts (M3×L4×OD4.2) and M3 screw kit | | In cart, for the chassis |
| M5 bolts | | For roller axles and shaft *(TBD)* |
| Bevel / miter gears, 1:1, 5 mm bore | 3 pairs | TBD, print or buy |
| Wheels or tires, ~30 mm | 3 | To buy |
| Thrust washers, shaft collars | 3 sets | To buy (nylon or PTFE washers) |
| Small parts | | To buy: power switch, ~5 A fuse, battery connector, Schottky diode on BEC output, battery sense divider (100 kΩ / 47 kΩ), 4.7 kΩ pull-ups, 1 kΩ series resistors, 470–1000 µF bulk caps near the drivers, 0.1 µF ceramics (7), wire, headers or perfboard |
| Panel magnets | | Owned. See section 8 for placement |
| 2D lidar, LDROBOT LD06 | 1 | To buy. Streams over UART (RX only, GPIO15). Check whether it needs a PWM motor-speed input, the baud rate (about 230400), and that the listing is a genuine LD06 |

Not buying: metal pinions (all gears are printed), steel rod (shafts already on hand, but the plan does not use them).

## 7. Power

```
2S LiPo (6.4–8.4 V) --+-- fuse/switch --+-- DRV8833 VM (motors, battery-direct)
                                        +-- BEC -> 5 V -> ESP32 VIN, navX
                                                    ESP32 3V3 -> AS5600, mux, encoders
```

- Motors are 6 V parts on a 7.4–8.4 V pack. The firmware caps PWM duty at `6 V / V_battery` (about 71% at 8.4 V), using the measured pack voltage. No separate buck regulator is needed. If speed varies too much as the pack drains, add a 6 V buck later.
- The battery sense divider doubles as a low-voltage cutoff (stop motors below ~6.4 V).
- Estimated 5 V rail without lidar: ESP32 0.25–0.5 A, navX ~0.1 A, sensors ~0.05 A. With lidar add about 0.3–0.4 A. Either fits within the 3 A BEC.
- Tie motor and logic grounds at one point.
- All ESP32 pins are 3.3 V only and not 5 V tolerant. Power every sensor and encoder from 3.3 V.

## 8. Sensors and bus layout

- **AS5600 x3:** one I²C bus through the TCA9548A, one AS5600 per channel. Sample at ≥ 500 Hz, filter the derivative. Estimated ~0.8 ms per cycle for three reads at 400 kHz *(confirm on bench)*.
- **navX2:** I²C address 0x32 *(verify)* does not collide with 0x36 or 0x70, so it sits on the main bus next to the mux. Read it at 50–100 Hz. There is no ready-made ESP32 library as far as I know, so expect to read its registers yourself.
- **Lidar (LDROBOT LD06):** UART receive only on GPIO15, since it streams by itself. It may need a PWM line for motor speed, so check the datasheet. If it does, GPIO0 or GPIO12 are the candidates. About 4.5k points/s, around 450 points per scan at 10 Hz. Either parse on the ESP32 and publish `LaserScan` through micro-ROS (needs the message buffers enlarged), or forward the raw serial stream to the laptop and parse there.
- **Mux pull-ups:** check for pull-ups on the upstream and each downstream channel. Do not stack many parallel pull-ups.
- **Magnet interference:** keep panel magnets several centimeters away from each AS5600, the navX (it has a magnetometer), and the N20 encoder motors. Put panel magnets around the outer edge of the chassis. Test the AS5600 angle and the navX yaw with the panels on and off.

## 9. Decisions log

- **3 modules, not 4.** A 4th module needs 28 signals, which doesn't fit a classic ESP32, plus more parts and weight. Chassis keeps room for a 4th. Revisit after one module works.
- **ESP32 only, no Pi or Pico.** Saves the Pi's power draw on an 800 mAh pack. Arduino Nano rejected (pins, interrupts, RAM).
- **DRV8833 over TB6612.** Smaller, 2 pins per channel, lower on-resistance. Needs bulk capacitance for voltage spikes (10.8 V max).
- **Battery-direct motor power with a duty cap** instead of a 6 V buck.
- **Lidar chosen over camera/AprilTag.** Processing is on the laptop either way.
- **Ring-gear (internal+external teeth) drive rejected.** It would need very large bearings or a roller ring, and it keeps the same coupling term anyway.
- **Coaxial chosen over differential swerve.** Diffy needs encoders on all 6 motors (12 inputs, about 28 signals in total), which doesn't fit the classic ESP32 or the motors already owned. Its main benefit, torque sharing, matters less here because friction, not motor torque, is the expected limit.
- **625ZZ roller turret** for the sleeve, chosen so a single reusable bearing size covers the whole module. Replaced the 688/683, 6806/608, and 6804 options.
- **All gears printed, heat-set inserts for set screws.** Metal pinions rejected.
- **LDROBOT LD06 chosen as the lidar.** About 40 g and 39 mm across, streams by itself (RX only), so it fits the classic ESP32 with no extra pins. Rejected: the RPLIDAR A1M8 (about 170 g and 96 mm across, needs a TX line and a motor line), a DIY spinning or swinging single-point rangefinder (the sensor's sample rate limits points per scan and gives sparse, smeared scans), and an I²C ToF ring (obstacle detection only, no mapping). The WitMotion dTOF listing appears to be the same LDROBOT family (D500 / LD19 class).
- **Classic ESP32 kept as the controller.** The ESP32-S3, a Pico or Pico 2 W as main board, a Nano encoder bridge, and sharing UART0 with the lidar through switches were all considered. None is needed now that the lidar is RX only.

## 10. Open questions

1. Gear tooth counts and center distances (FeatureScript).
2. Bevel arrangement and the sign of the coupling term.
3. Stall current and torque of the chosen N20 against the DRV8833 limit.
4. Bevel / miter gears with a 5 mm bore: print or buy.
5. Inner shaft material, and the wheel axle diameter and bearing size.
6. Roller track material for the sleeve, and the preload method (slot or eccentric).
7. Whether the fork can fit a 1.5:1 bevel pair, or whether drive stays at 1:1.
8. Whether the steer reduction can fit in 40 mm of gear, or needs two stages.
9. Whether the encoders on the GA12-N20 run on 3.3 V.
10. Whether the LD06 needs a PWM motor-speed input (a pin from GPIO0 or GPIO12), or can be tied low or left to internal control.

## 11. Milestones

1. Onshape model of the four moving bodies with revolute and gear relation mates. Confirm the coupling direction.
2. Roller turret test piece: sleeve, rollers, inner shaft and bearings only. Spin by hand and check for binding.
3. Python kinematics + simulation, including the coupling term and the stopped-wheel steer case.
4. Single-module bench rig: steer loop and drive loop.
5. Three modules + ESP32 firmware + micro-ROS link.
6. navX, odometry, teleop.
7. Lidar + `slam_toolbox` on the laptop, when the budget allows.
