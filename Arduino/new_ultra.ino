// -----------------------------
// Pin Definitions
// -----------------------------
#define alarm 7
#define red_light 6
#define green_light 5
#define motorA1 2
#define motorA2 3
#define motorB1 8
#define motorB2 9
#define speedMotor 10

// Ultrasonic Pins
const int trigPinM = A1;
const int echoPinM = A0;
const int trigPinR = A3;
const int echoPinR = A2;
const int trigPinL = A4;
const int echoPinL = A5;

// Encoder Pins
#define leftEncoderPin 18
#define rightEncoderPin 19
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// -----------------------------
// State Variables
// -----------------------------
int threshold_dist = 50;
int distM, distR, distL;
String inputString = "";
bool stringComplete = false;

unsigned long lastCommandTime = 0;
bool manual_override = false;
const unsigned long overrideTimeout = 1000; // 1 second

// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(9600);
  inputString.reserve(50);

  pinMode(alarm, OUTPUT);
  pinMode(red_light, OUTPUT);
  pinMode(green_light, OUTPUT);

  pinMode(trigPinM, OUTPUT); pinMode(echoPinM, INPUT);
  pinMode(trigPinR, OUTPUT); pinMode(echoPinR, INPUT);
  pinMode(trigPinL, OUTPUT); pinMode(echoPinL, INPUT);

  pinMode(motorA1, OUTPUT); pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT); pinMode(motorB2, OUTPUT);
  pinMode(speedMotor, OUTPUT);

  analogWrite(speedMotor, 0);

  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), countRight, RISING);
}

// -----------------------------
// Main Loop
// -----------------------------
void loop() {
  unsigned long currentMillis = millis();

  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
    lastCommandTime = currentMillis;
    manual_override = true;
  }

  if (manual_override && (currentMillis - lastCommandTime > overrideTimeout)) {
    manual_override = false;
  }

  if (!manual_override) {
    readUltrasonics();  // Autonomous obstacle avoidance
  }

  sendEncoderData();
  delay(100); // loop pacing
}

// -----------------------------
// Serial Event
// -----------------------------
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// -----------------------------
// Process Incoming Command
// -----------------------------
void processSerialCommand(String cmd) {
  cmd.trim(); // remove newline/whitespace

  if (cmd.length() == 4) {
    char dir = cmd.charAt(0);
    int speed = cmd.substring(1).toInt();
    speed = constrain(speed, 0, 255);
    analogWrite(speedMotor, speed);

    switch (dir) {
      case 'f': Lmotor_forward(); Rmotor_forward(); break;
      case 'b': Lmotor_reverse(); Rmotor_reverse(); break;
      case 'l': Lmotor_reverse(); Rmotor_forward(); break;
      case 'r': Lmotor_forward(); Rmotor_reverse(); break;
      case 's':
      default:
        analogWrite(speedMotor, 0);
        Lmotor_stop(); Rmotor_stop();
        break;
    }
  }
  else if (cmd.startsWith("CMD:")) {
    int sep = cmd.indexOf(':', 4);
    if (sep > 4) {
      int leftDir = cmd.substring(4, sep).toInt();
      int rightDir = cmd.substring(sep + 1).toInt();
      setMotorDirection(leftDir, rightDir);
    }
  }
}

// -----------------------------
// Obstacle Avoidance
// -----------------------------
void readUltrasonics() {
  distM = ultrasonic(trigPinM, echoPinM);
  distR = ultrasonic(trigPinR, echoPinR);
  distL = ultrasonic(trigPinL, echoPinL);

  if (distM < threshold_dist && distR < threshold_dist - 20) {
    Rmotor_forward(); Lmotor_reverse();
  } else if (distM < threshold_dist && distL < threshold_dist - 20) {
    Lmotor_forward(); Rmotor_reverse();
  } else if (distM < threshold_dist) {
    Rmotor_stop(); Lmotor_stop(); Rlight_on(); Glight_off();
  } else {
    Glight_on(); Rlight_off(); alarm_off();
  }
}

// -----------------------------
// Encoder Output
// -----------------------------
void sendEncoderData() {
  Serial.print("ENC_L:"); Serial.print(leftTicks);
  Serial.print(" ENC_R:"); Serial.println(rightTicks);
}

// -----------------------------
// Ultrasonic Measurement
// -----------------------------
int ultrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

// -----------------------------
// Motor Controls
// -----------------------------
void Rmotor_forward()  { digitalWrite(motorB1, HIGH); digitalWrite(motorB2, LOW); }
void Rmotor_reverse()  { digitalWrite(motorB1, LOW);  digitalWrite(motorB2, HIGH); }
void Rmotor_stop()     { digitalWrite(motorB1, LOW);  digitalWrite(motorB2, LOW); }

void Lmotor_forward()  { digitalWrite(motorA1, HIGH); digitalWrite(motorA2, LOW); }
void Lmotor_reverse()  { digitalWrite(motorA1, LOW);  digitalWrite(motorA2, HIGH); }
void Lmotor_stop()     { digitalWrite(motorA1, LOW);  digitalWrite(motorA2, LOW); }

// -----------------------------
// Lights and Alarm
// -----------------------------
void Glight_on()       { digitalWrite(green_light, HIGH); }
void Glight_off()      { digitalWrite(green_light, LOW); }
void Rlight_on()       { digitalWrite(red_light, HIGH); }
void Rlight_off()      { digitalWrite(red_light, LOW); }
void alarm_on()        { digitalWrite(alarm, HIGH); }
void alarm_off()       { digitalWrite(alarm, LOW); }

// -----------------------------
// Encoder ISRs
// -----------------------------
void countLeft()  { leftTicks++; }
void countRight() { rightTicks++; }

// -----------------------------
// Raw Motor Direction Control
// -----------------------------
void setMotorDirection(int leftDir, int rightDir) {
  if (leftDir > 0) Lmotor_forward();
  else if (leftDir < 0) Lmotor_reverse();
  else Lmotor_stop();

  if (rightDir > 0) Rmotor_forward();
  else if (rightDir < 0) Rmotor_reverse();
  else Rmotor_stop();
}
