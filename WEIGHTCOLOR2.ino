#include <Servo.h>
#include <Adafruit_NAU7802.h>

Adafruit_NAU7802 nau;


int redFreq = 0, greenFreq = 0, blueFreq = 0;
int32_t weight = 0;

//Servo definitions
Servo bottom1;
Servo bottom2;
Servo top;
Servo wheel1;
Servo wheel2;
Servo wheel3;
int cdelay = 100;

const int S0 = 6;
const int S1 = 5;
const int S2 = 4;
const int S3 = 3;
const int sensorOut = 2;

void setup() {
  Serial.begin(115200);
  delay(8000);
  Serial.println("Starting system...");
  bottom1.attach(8);
  bottom2.attach(9);
  top.attach(10);
  wheel1.attach(11);
  wheel2.attach(12);
  wheel3.attach(7);
top.write(0);
bottom1.write(0);
bottom2.write(0);


if (!nau.begin()) {
    Serial.println("Failed to find NAU7802");
    while (1) delay(10);
}
  nau.setLDO(NAU7802_3V3);
  nau.setGain(NAU7802_GAIN_128);
  nau.setRate(NAU7802_RATE_10SPS);

  for (uint8_t i=0; i<10; i++) {
    while (!nau.available()) delay(1);
    nau.read();
}
  while (!nau.calibrate(NAU7802_CALMOD_INTERNAL)) { delay(500); }
  while (!nau.calibrate(NAU7802_CALMOD_OFFSET))   { delay(500); }

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);

  Serial.println("=== Combo Rules ===");
  Serial.println("TT Accept: Orange, White | Reject: Black, Yellow");
  Serial.println("GP Accept: Yellow        | Reject: Black, White");
  Serial.println("SB -> handled by Python dot detection");
  Serial.println("====================");
}

void rejectpush(){
  wiggle(wheel1);
  bottom1.write(140);
  wiggle(wheel1);
  wiggle(wheel1);
  wiggle(wheel1);
  bottom1.write(0);
  delay(100);
};

void accept(){
  wiggle(wheel2);
  bottom2.write(140);
  wiggle(wheel2);
  wiggle(wheel2);
  wiggle(wheel2);
  bottom2.write(0);
  delay(1000);
};

void pushBall() {
  Serial.println("Servo: Push ball forward");
  top.write(0); delay(500); top.write(180); delay(500);top.write(0);
}

void x(){
    wheel2.write(140);
    wheel1.write(40);
    delay(1500);  // Adjust speed (smaller = faster)
    wheel2.write(90);
    wheel1.write(90);
    delay(250);
};

void y(){
    wheel3.write(40);
    wheel1.write(140);
    delay(1500);  // Adjust speed (smaller = faster)
    wheel3.write(90);
    wheel1.write(90);
    delay(250);
};

//void squashActionX() {
  //servo2.write(60); servo1.write(120);
  //delay(1500); servo1.write(90); servo2.write(90); delay(250);
//}
//void squashActionY() {
  //servo1.write(40); servo3.write(140);
  //delay(1500); servo1.write(90); servo3.write(90); delay(250);
//}
// === Continuous wheel spinning ===
void sidesStart() {
  // spin all relevant wheels to rotate the ball
  x();
  x();
  x();
  x();
  y();
  y();
}

void sidesStop() {
  // stop all rotation
  wheel1.write(90);
  wheel2.write(90);
  wheel3.write(90);
}

//void sides(){
  //  x();
    //x();
    //x();
    //x();
    //y();
  //  y();
//};

String weightsensor(){
  Serial.println("Reading weight...");

  // Wait until ball lands (weight > threshold)
  weight = nau.read();
  while (weight < 20000){
    delay(100);
    weight = nau.read();
    // Serial.println(weight);
  }

  // Let it settle after impact
  delay(400);

  // Check stability: repeat until stable
  bool stable = false;
  const int tolerance = 1000;  // allowed fluctuation
  const int samples = 20;

  while (!stable) {
    int32_t sum = 0;
    int32_t first = nau.read();
    bool allClose = true;

    for (int i=0; i<samples; i++) {
      int32_t val = nau.read();
      sum += val;
      if (abs(val - first) > tolerance) {
        allClose = false;  // unstable
      }
      delay(50);
    }

    if (allClose && (sum/samples) > 15000) {
      weight = sum / samples;
      stable = true;
    } else {
      Serial.print("Weight unstable, retrying...");
      Serial.println(weight);
      delay(200);
    }
  }

  // Classification
  String result = "NONE";
  if ((weight >= 20000) && (weight <= 42000)){
    result = "TT";  // Table Tennis
  }
  else if ((weight >= 43000) && (weight <= 90000)){
    result = "GP";  // Golf Practice
  }
  else if ((weight > 200000)){
    result = "SB";  // Squash Ball
  }

  Serial.print("Stable weight = ");
  Serial.print(weight);
  Serial.print(" -> Expected: ");
  Serial.println(result);

  return result;
};

void wiggle(Servo motor){
  motor.write(180);
  delay(cdelay);
  motor.write(0);
  delay(cdelay);
  motor.write(180);
  delay(cdelay);
  motor.write(0);
  delay(cdelay);
  motor.write(180);
  delay(cdelay);
  motor.write(0);
  delay(cdelay);
  motor.write(90);

};

String coloursensor() {
  String colour = "NO COLOUR";

  // --- Read RGB values ---
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  delay(100);
  redFreq = pulseIn(sensorOut, LOW);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  delay(100);
  greenFreq = pulseIn(sensorOut, LOW);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  delay(100);
  blueFreq = pulseIn(sensorOut, LOW);

  // --- Print for debugging ---
  Serial.print("R: "); Serial.print(redFreq);
  Serial.print("  G: "); Serial.print(greenFreq);
  Serial.print("  B: "); Serial.println(blueFreq);

  // === COLOR DETECTION BASED ON 50mm SENSOR VALUES ===

  // Orange TT
  if ((redFreq > 7 && redFreq < 15) &&
      (greenFreq > 30 && greenFreq < 45) &&
      (blueFreq > 30 && blueFreq < 50)) {
    colour = "O"; // Orange Table Tennis
  }

  // White TT
  else if ((redFreq > 4 && redFreq < 9) &&
           (greenFreq > 13 && greenFreq < 30) &&
           (blueFreq > 13 && blueFreq < 29)) {
    colour = "W"; // White Table Tennis
  }

  // Yellow GP
  else if ((redFreq > 8 && redFreq < 13) &&
           (greenFreq > 22 && greenFreq < 30) &&
           (blueFreq > 22 && blueFreq < 30)) {
    colour = "Y"; // Yellow Golf Practice
  }

  // Black SB
  else if ((redFreq > 25 && redFreq < 600) &&
           (greenFreq > 100 && greenFreq < 600) &&
           (blueFreq > 100 && blueFreq < 520)) {
    colour = "B"; // Black Squash
  }

  Serial.print("Detected colour: ");
  Serial.println(colour);
  return colour;
}

void loop() {
  String type = weightsensor();
  pushBall();
  delay(5000);
    String colour = coloursensor();
  delay(1000);
  int reject = 1;
// --- Normal rules ---
  if(type == "TT"){
    if(colour == "O" || colour == "W"){ reject = 0; }
    if(colour == "B"){ reject = 1; }
    if(colour == "Y"){ 
      Serial.println("Yellow detected but TT type -> ACCEPT because probably GP ball");
      reject = 0; 
    }
  }
  if(type == "GP"){
    if(colour == "Y"){ 
      Serial.println("Yellow detected and GP type -> ACCEPT");
      reject = 0; 
    }
    if(colour == "B" || colour == "W"){ reject = 1; }
  }
 // if(type == "TT" && (colour=="O"||colour=="W")) reject = 0;
 // if(type == "GP" && colour=="Y") reject = 0;
 // if(type == "SB" && colour=="B") reject = 0;
 // if(type == "SB" && colour=="Y") reject = 1;
 if(type == "SB"){
    if(colour == "B"){ reject = 0; }
    if(colour == "Y"){ reject = 1; }
    if(colour == "W"){ reject = 1; }
  }

  // --- Fallback for Orange TT ---
  if(type == "GP" && colour == "O"){
    Serial.println("Fallback -> TT assumed ORANGE");
    colour = "O";
    reject = 0;
  }

  // --- Show combo ---
  //Serial.print("Combo detected: ");
  //Serial.print(type);
  //Serial.print(" + ");
  //Serial.println(colour);

  // --- Result ---
  if (reject == 1){
    Serial.println("Result: REJECT");
  } else {
    Serial.println("Result: ACCEPT");
  }

 // Serial.println("---------------------");
 // delay(2000); // wait for next ball
//}

  Serial.print("Combo detected: "); Serial.println(type + " + " + colour);

  if (reject == 1 && type != "SB") {
    Serial.println("Result: REJECT");
    rejectpush();
  } 
  else if (reject == 0 && type != "SB") {
    Serial.println("Result: ACCEPT");
    accept();
  } 
  else if (type == "SB" && colour != "B"){
    Serial.println("Result: REJECT");
    rejectpush();
  }
else if (type == "SB" && colour == "B") {
    Serial.println("Squash ball detected -> requesting Python dot detection...");
    Serial.println("START_SQUASH");   // Trigger Python
    Serial.println("Rotating Ball for Dot Recognition...");

    sidesStart();   // Start continuous rotation

    unsigned long lastStatus = millis();
    bool recognized = false;

    while (!recognized) {
      // keep the wheels running
      sidesStart();

      // Periodically print status (every 3s)
      if (millis() - lastStatus > 3000) {
        Serial.println("...Waiting for Python detection...");
        lastStatus = millis();
      }

      // Check if Python has sent a dot detection result
      if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.startsWith("SB_")) {
          Serial.print("Got squash dot info: ");
          Serial.println(cmd);

          sidesStop();   // Stop spinning
          accept();      // Accept the ball
          recognized = true;
        }
      }
    }
  }
}
