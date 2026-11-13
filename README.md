# 5-DOF Robotic Arm with Dual Ultrasonic Sensors

An Arduino-controlled robotic arm that finds an object on the table, picks it up and drops it at a fixed spot, without any human input. A small Processing app shows the sensor readings live as a radar screen.

**Team project** (5 people, undergraduate Mechanical Engineering, Cyprus University of Technology, December 2025).
My part: wiring of the servos, sensors and power supply, plus the market analysis in the project report. The code was written as a team, with one teammate leading it.

## How it works

The arm is a 5-joint kit (base, shoulder, elbow, wrist, gripper) that came with no code, wiring or power setup. We built everything around it:

- **Arduino Mega 2560** runs the control logic.
- **PCA9685** board drives the 5 servos over I2C at 50 Hz.
- **Two HC-SR04 ultrasonic sensors**: one on the shoulder looking forward, one near the gripper looking down.
- **3D-printed mounts** for the sensors and the controller board (SolidWorks files in `cad/`).

The code is a state machine with five steps that repeat forever:

1. **Coarse scan.** The base sweeps from 0° to 150° in 3° steps. The shoulder sensor looks for anything between 10 and 35 cm away.
2. **Ready pose.** The arm turns to that angle and picks one of three preset poses (near, mid, far) based on the distance.
3. **Fine scan.** The base sweeps ±30° around the first angle in 2° steps, using the gripper sensor. The object must stay under 10 cm for 250 ms before the arm accepts it, which filters out noisy readings.
4. **Pick up.** Shoulder and elbow move down, the gripper closes, and the arm lifts.
5. **Place.** The base turns to 180°, the gripper opens, and the arm goes back home.

Each distance is the average of 5 sensor readings, and readings with no echo are thrown away. Servo moves go one degree at a time so the arm doesn't jerk.

While it runs, the Arduino sends a line like `S1,12.3,S2,24.8,BA,90` over serial every 100 ms (gripper distance, shoulder distance, base angle). The Processing sketch reads these lines and draws the shoulder sensor's hits on a radar screen.

## Results

The arm found, picked up and placed small objects on its own across repeated runs. The second scan still had small angle errors in some runs, so the code would need more tuning for full accuracy. The "far" pose (28 to 40 cm) did not work with our test object. It is still in the code but was not used in practice.

## Repository structure

```
arduino/robotic_arm/robotic_arm.ino   main control code
processing/radar_view/radar_view.pde  live radar display
cad/                                  SolidWorks parts for the 3D-printed mounts
```

## How to run

**Arduino**
1. Install the Arduino IDE and the **Adafruit PWM Servo Driver Library** (Library Manager).
2. Open `arduino/robotic_arm/robotic_arm.ino`, select **Arduino Mega 2560**, and upload.
3. Wiring: PCA9685 on I2C (SDA/SCL), servos on channels 0 to 4 (gripper, wrist, elbow, shoulder, base). Gripper sensor on pins 7 (trig) and 6 (echo), shoulder sensor on pins 8 (trig) and 9 (echo). The servos need their own power supply, not the Arduino 5V pin.
4. Open the Serial Monitor at 115200 baud to see the scan log.

**Processing radar (optional)**
1. Install Processing 4 and open `processing/radar_view/radar_view.pde`.
2. Close the Arduino Serial Monitor first, since only one program can use the port.
3. If the radar stays on "Waiting for serial data...", change `Serial.list()[0]` to the index of your Arduino port (the sketch prints the list at startup).

## Tools

Arduino (C++), Processing, SolidWorks, 3D printing.
