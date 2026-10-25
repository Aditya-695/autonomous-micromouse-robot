# Micromouse Maze Solver Robot

This is my Arduino-based Micromouse project. The goal is to build a small robot that senses the maze, keeps track of walls, and chooses its next move without being manually steered.

The robot uses an Arduino UNO R4 Minima, three VL53L0X distance sensors, a TB6612FNG motor driver, and two N20 geared motors with encoders. The firmware combines encoder feedback, PID-style wheel balancing, and a Flood Fill distance map to choose a route through an 8 × 8 maze.

Build and testing note:The code is a hardware-specific starting point. Encoder pin assignments, encoder counts per cell/turn, sensor offsets, PID gains, and motor directions must be calibrated on the actual robot. Do not describe the robot as fully validated until it has completed repeatable maze tests.

## Features

* Three ToF sensors for front, left, and right wall detection.
* Sequential XSHUT startup so each VL53L0X sensor can be assigned a unique I2C address.
* Encoder input from both motors.
* Encoder-feedback movement intended to travel one cell at a time.
* PID-style correction based on left/right encoder count difference.
* An 8 × 8 maze representation.
* Flood Fill distance-map calculation from the goal.
* Direction selection using known wall information.
* Timeouts and a stop response for sensor errors or movement failures.

## Hardware

* Arduino UNO R4 Minima
* 3 × VL53L0X ToF distance sensors
* TB6612FNG dual H-bridge motor driver
* 2 × N20 geared DC motors with quadrature encoders
* 3S Li-ion battery pack
* LM2596 buck converter
* Chassis, wheels, and wiring

Check the voltage/current ratings for the exact modules used. A conventional 3S Li-ion pack is about 11.1 V nominal and 12.6 V fully charged. Set the buck converter before connecting the electronics, and use a common ground.

## Pin map

### TB6612FNG

|Arduino|Signal|
|D6|AIN1|
|D7|AIN2|
|D9|PWMA (left motor PWM)|
|D10|BIN1|
|D12|BIN2|
|D11|PWMB (right motor PWM)|
|D13|STBY|

### ToF sensors

|Sensor|XSHUT|Address|
|Front|A0|`0x30`|
|Left|A1|`0x31`|
|Right|A2|`0x32`|

### Encoders

The example firmware uses:

|Encoder signal|Arduino pin|
|Left A|D2|
|Left B|D4|
|Right A|D3|
|Right B|D5|



## How it works

1. At startup, each ToF sensor is enabled separately using its XSHUT pin and assigned a different I2C address.
2. The robot reads front, left, and right distances.
3. Wall observations update the current cell's wall map.
4. Flood Fill assigns each reachable cell a distance value from the goal.
5. The robot chooses an open neighboring cell with a lower Flood Fill value, preferring left, straight, right, and then reverse when values are equal.
6. The heading is updated after an encoder-counted turn.
7. Both wheel encoders are read while moving toward the next cell. A PID-style correction adjusts the two motor commands to reduce the difference between encoder counts.
8. The robot updates its cell coordinates only after the movement routine reports success.

The maze map starts unknown except for the outer boundary. The algorithm can only choose routes using walls it has already observed; it does not know the entire maze in advance.

## PID and encoder calibration

The values in the sketch are starting placeholders, not universal values:

* `TICKS\_PER\_CELL\_LEFT`
* `TICKS\_PER\_CELL\_RIGHT`
* `TICKS\_PER\_90\_LEFT`
* `TICKS\_PER\_90\_RIGHT`
* `Kp`, `Ki`, and `Kd`
* `BASE\_PWM`, `TURN\_PWM`, and `MAX\_PWM`
* `WALL\_THRESHOLD\_MM`, sensor offsets, and `FRONT\_STOP\_MM`

To calibrate:

1. Raise the robot so the wheels can spin safely and confirm both encoder counts change.
2. Check encoder signs while moving the robot forward; fix the ISR sign or wiring if one count decreases.
3. Measure encoder ticks for a straight 250 mm move and set the cell constants.
4. Measure ticks for a 90-degree in-place turn and set turn constants.
5. Start with low PWM and tune `Kp`; add `Kd` only as needed. Keep `Ki` small unless there is a demonstrated steady-state error.
6. Test one-cell moves and turns on the floor before entering the maze.

## Build and upload

1. Install the Arduino IDE.
2. Install Arduino UNO R4 board support in Boards Manager.
3. Install the **VL53L0X** library by Pololu in Library Manager.
4. Open `src/micromouse.ino`.
5. Verify the board, pin map, encoder wiring, and power connections.
6. Compile and upload.
7. First test with the wheels raised, then test at low speed in a clear area.
8. Only run in the maze after calibrating the movement and turns.

## Current limitations and safety

This firmware has not been verified against your physical robot in this response. Treat it as an integration starting point, not a guaranteed fully functional build. Real-world behavior depends on the encoder resolution, encoder mounting, interrupt support, sensor alignment, motor direction, cell-centering mechanics, and PID tuning.

The example updates wall data from three sensors at the current cell and uses a simplified cell-to-cell motion model. It does not include a complete exploration/return/run-speed strategy, diagonal motion, or robust handling of every possible maze geometry. Validate map updates and motion carefully before relying on its route choices.

Keep the wheels off the ground during initial motor/encoder tests. Disconnect power before changing wiring. Do not power motors from GPIO pins.

## Future improvements

* Add a reliable center-of-cell alignment routine.
* Add encoder velocity measurement and separate wheel-speed PID loops.
* Improve wall confidence handling with multiple readings.
* Add a maze visualization/debug mode.
* Add a fast second run after maze exploration.
* Save and inspect maze maps between runs.

## Author

**Aditya Jangir**  
B.Tech, Electronics Internet of Things  
J.C. Bose University of Science and Technology, YMCA, Faridabad

* GitHub: https://github.com/Aditya-695
* LinkedIn: https://www.linkedin.com/in/aditya-jangir-b303a92a0

