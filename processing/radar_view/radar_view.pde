import processing.serial.*;

Serial myPort;

// Latest values from Arduino
float dGrip = 0;        // S1 - grip sonar (cm) 
float dShoulder = 0;    // S2 - shoulder sonar (cm)
float baseAngle = 90;   // BA - base servo angle (deg)

// Smoothed display values
float dispShoulder = 0;
float dispBaseAngle = 90;

boolean gotData = false;   // becomes true once we get first packet

// Visualization parameters
float maxRangeCm  = 50.0;    // max range for this radar (cm)
float radarRadius = 260;     // radius in pixels of radar semicircle

void setup() {
  size(900, 650);
  background(0);

  // List available serial ports
  println("Available serial ports:");
  printArray(Serial.list());

  // IMPORTANT:
  // Change [0] to the index that corresponds to your Arduino COM port.
  myPort = new Serial(this, Serial.list()[0], 115200);

  // Buffer until newline, each line matches one telemetry packet
  myPort.bufferUntil('\n');
}

void draw() {
  background(0);

  // If we never received data yet, show status
  fill(255);
  textSize(16);
  if (!gotData) {
    text("Waiting for serial data...", 20, 30);
    return;
  }

  // -- Smooth animation (low-pass filter) --
  float alpha = 0.25;  // smoothing factor
  dispShoulder  += alpha * (dShoulder  - dispShoulder);
  dispBaseAngle += alpha * (baseAngle - dispBaseAngle);

  // -- Text info --
  fill(255);
  textSize(18);
  text("Shoulder sonar (S2): " + nf(dShoulder, 1, 1) + " cm", 20, 30);
  text("Base angle (BA):     " + nf(baseAngle,   1, 1) + " deg", 20, 55);

  // (Still parse dGrip from serial but do NOT draw it here)
  text("Grip sonar (S1):     " + nf(dGrip, 1, 1) + " cm  (not shown on this radar)", 20, 80);

  // -- Draw radar --
  pushMatrix();
  
  // Place radar center at bottom-center of the window
  translate(width/2, height - 60);

  // Draw radar semicircle (0..maxRangeCm) as PI..2*PI
  noFill();
  stroke(0, 200, 0);
  strokeWeight(2);

  // Concentric rings (25%, 50%, 75%, 100% of max range)
  for (int i = 1; i <= 4; i++) {
    float r = radarRadius * i / 4.0;
    arc(0, 0, 2*r, 2*r, PI, TWO_PI);
  }

  // Center vertical line (straight ahead)
  line(0, 0, 0, -radarRadius);

  // Small ticks for left/right boundaries (0° and 180°)
  line(-radarRadius, 0, -radarRadius + 10, -10); // left boundary tick
  line(radarRadius, 0, radarRadius - 10, -10);   // right boundary tick

  // -- Convert baseAngle to radar angle --
  // baseAngle: 0° = left, 180° = right
  // radar: left = PI rad, right = 0 rad
  float theta = map(dispBaseAngle, 0, 180, PI, 0);

  // -- Shoulder sonar "object" ping --
  float dS_clamped = constrain(dispShoulder, 0, maxRangeCm);
  float rS = map(dS_clamped, 0, maxRangeCm, 0, radarRadius);

  // Only draw the dot if the object is within our max range
  if (dShoulder > 0 && dShoulder <= maxRangeCm) {
    float sx = rS * cos(theta);
    float sy = -rS * sin(theta);

    noStroke();
    fill(255, 255, 0);   // yellow for detected object (shoulder sonar)
    ellipse(sx, sy, 16, 16);
  }

  // --- Base scan direction line ---
  stroke(0, 200, 200);
  strokeWeight(2);
  line(0, 0, radarRadius * cos(theta), -radarRadius * sin(theta));

  popMatrix();
}

// ------------------------------------------------------
// Serial event: parse line "S1,<grip>,S2,<shoulder>,BA,<angle>"
// ------------------------------------------------------
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;

  line = trim(line);
  if (line.length() == 0) return;

  // Expected format: S1,xx.x,S2,yy.y,BA,zz
  String[] parts = split(line, ',');
  if (parts.length < 6) {
    // not a valid packet
    return;
  }

  // Basic sanity check of labels
  if (!parts[0].equals("S1") || !parts[2].equals("S2") || !parts[4].equals("BA")) {
    return;
  }

  try {
    dGrip     = float(parts[1]);  // currently only shown as text
    dShoulder = float(parts[3]);
    baseAngle = float(parts[5]);

    gotData = true;
  } 
  catch (Exception e) {
    // ignore parse errors
    println("Parse error on line: " + line);
  }
}