# CitrusSwerve: Design Doc

3-module coaxial swerve drivetrain. Small scale (N20 gearmotors), one ESP32 on the robot, ROS 2 on a laptop.
Status: **draft v0.2**. Values marked *(TBD)* are placeholders to be set in CAD or testing. Items marked *(verify)* come from memory and must be checked against datasheets.

## 1. Architecture

```
 Laptop: ROS 2 (micro-ROS agent, slam_toolbox, teleop, rviz)
        ^
        | WiFi (micro-ROS over UDP)
        v
 ESP32 (on robot)
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

## 2. Module mechanical concept (coaxial)

Both motors sit on the fixed chassis, so no wires cross the rotating joint and steering can rotate continuously.

- **Steer path:** steer N20 -> 3:1 reduction -> gear on a hollow outer shaft -> rotates the module (fork + wheel).
- **Drive path:** drive N20 -> 1:1 -> inner shaft (concentric inside the outer shaft) -> bevel gear A -> bevel gear B on the wheel axle (1:1 miter pair assumed).
- **Steering sensor:** AS5600 reads a diametric magnet on a small 1:1 gear meshed with the steer output gear. An equal-tooth gear keeps the reading absolute over 360°. The sensor is fixed to the chassis. *(Alternative: radially magnetized ring magnet on the hollow shaft.)*
- Gear geometry comes from the Onshape gear FeatureScript. *(TBD: module, tooth counts, center distances.)*

## 3. Steer-drive coupling

With the inner shaft held still and the module rotating, bevel gear B walks around bevel gear A and the wheel rolls. Wheel rotation depends on the drive shaft angle *relative to the module*:

```
ω_wheel = r · (ω_drive − ω_steer)
```

`r` is the total ratio from relative inner-shaft rotation to wheel rotation (1.0 here with 1:1 drive and a 1:1 bevel pair). The sign depends on the bevel arrangement. **Verify in CAD or on the bench.**

To get a desired wheel speed while steering:

```
ω_drive_cmd = ω_wheel_cmd / r + ω_steer
```

- `ω_steer` is the module rate (steer motor rate / 3), from the filtered AS5600 derivative, plus the commanded steer rate as feedforward.
- Odometry must use `r(ω_drive − ω_steer)`, not the drive encoder alone.
- **Headroom:** the drive motor must also cover the steer rate. If `ω_drive_cmd` exceeds the motor's limit, reduce wheel speed and keep steering priority.

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

## 5. Motors and design targets

Drive and steer use the same part, a **500 rpm, 6 V N20**. The drive motors have the encoder version. This gives one SKU and shared spares.

| Item | Value | Basis |
|---|---|---|
| Wheel diameter | ~30 mm *(TBD)* | |
| Drive ratio | 1:1 | |
| Wheel speed, no load | 500 rpm, ~0.79 m/s | |
| Wheel speed, loaded | ~350 rpm, ~0.55 m/s | assumes 70% of no-load |
| Usable speed while steering | ~0.45 m/s | coupling takes motor headroom |
| Steer ratio | 3:1 | |
| Module rate, loaded | ~117 rpm, 360° in ~0.5 s | assumes 70% of no-load |
| Steer accuracy | ≤ 1° | backlash dominated |
| Steer loop | ≥ 500 Hz | |
| Drive loop | ≥ 200 Hz | |

If drive feels sluggish under load, switch the drive motors to 300 rpm and keep 500 rpm for steering. Look up stall torque and stall current for the exact part before ordering.

## 6. Electronics (bill of materials)

| Part | Qty | Notes |
|---|---|---|
| ESP32 | 1 | Variant to be decided, see section 9 |
| DRV8833 | 3 | One per module (steer + drive). 10.8 V max, 1.5 A continuous per channel |
| N20 500 rpm 6 V, with encoder | 3 | Drive |
| N20 500 rpm 6 V, no encoder | 3 | Steer |
| AS5600 + diametric magnet | 3 | Fixed address 0x36, needs a mux or AS5600L |
| navX2-MXP | 1 | Yaw. Level shifting *(verify I/O is 3.3 V)* |
| 2D lidar (LD06 / LD19 class) | 1 | 5 V power, UART out at 3.3 V *(verify)* |
| 5 V 3 A BEC | 1 | Electronics only. Confirm input range covers 2S (8.4 V) |
| 2S LiPo, 800 mAh | 1 | Min 6.4 V cutoff, check C rating |
| TCA9548A I²C mux | 1 | Or use AS5600L parts |
| Small parts | | Power switch, ~5 A fuse, battery connector, battery sense divider, 470–1000 µF bulk caps near the drivers, 0.1 µF ceramic across each motor |

## 7. Power

```
2S LiPo (6.4–8.4 V) --+-- fuse/switch --+-- DRV8833 VM (motors, battery-direct)
                                        +-- BEC -> 5 V -> ESP32, lidar, navX
                                                    ESP32 3V3 -> AS5600, encoders, driver logic
```

- Motors are 6 V parts on a 7.4–8.4 V pack. The firmware caps PWM duty at `6 V / V_battery` (about 71% at 8.4 V), using the measured pack voltage. No separate buck regulator is needed. If speed varies too much as the pack drains, add a 6 V buck later.
- The battery sense divider doubles as a low-voltage cutoff (stop motors below ~6.4 V).
- Estimated 5 V rail: ESP32 0.25–0.5 A, lidar 0.3–0.4 A, navX ~0.1 A, sensors ~0.05 A. Total about 1–1.3 A of the 3 A available.
- Tie motor and logic grounds at one point.
- All ESP32 pins are 3.3 V only and not 5 V tolerant. Power every sensor and encoder from 3.3 V.

## 8. Sensors and bus layout

- **AS5600 x3:** one I²C bus through a TCA9548A mux (or AS5600L parts). Sample at ≥ 500 Hz, filter the derivative.
- **navX2:** I²C address 0x32 *(verify)* does not collide with 0x36, so it can sit upstream of the mux on the same bus. UART is the alternative. There is no ready-made ESP32 library as far as I know, so expect to read its registers yourself.
- **Lidar:** UART receive only. About 4.5k points/s, around 450 points per scan at 10 Hz. Either parse on the ESP32 and publish `LaserScan` through micro-ROS (needs the message buffers enlarged), or forward the raw serial stream to the laptop and parse there.

## 9. Open questions

1. **ESP32 variant and pin map.** Signals needed: 12 motor outputs, 6 encoder inputs, 2 I²C, 1 lidar RX, 1 battery ADC (and 2 more if navX uses UART). A classic ESP32 has only about 19 clean pins once you exclude flash pins, strapping pins and UART0, so this is tight. An ESP32-S3 has many more GPIOs and enough PCNT units (4) for 3 encoders. Check micro-ROS support for the chosen variant. **Resolved:** classic ESP32 DevKit V1. All 22 signals fit with zero spare pins, see `electrical/pinmap.md`. The navX therefore goes on the shared I²C bus, not UART.
2. Gear tooth counts and center distances (FeatureScript).
3. Bevel arrangement and the sign of the coupling term.
4. Stall current of the chosen N20 against the DRV8833 limit.
5. Whether lidar is parsed on the ESP32 or on the laptop.

## 10. Milestones

1. Python kinematics + simulation, including the coupling term.
2. Single-module bench rig: steer loop and drive loop.
3. Three modules + ESP32 firmware + micro-ROS link.
4. navX, odometry, teleop.
5. Lidar + `slam_toolbox` on the laptop.
