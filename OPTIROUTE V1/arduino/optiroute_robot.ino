const unsigned long BAUD_RATE = 9600;

const int LEFT_ENABLE = 5;
const int RIGHT_ENABLE = 6;
const int LEFT_IN1 = 7;
const int LEFT_IN2 = 8;
const int RIGHT_IN1 = 9;
const int RIGHT_IN2 = 10;
const int ULTRASONIC_ECHO = 11;
const int ULTRASONIC_TRIGGER = 12;
const int IR_SENSOR_PINS[] = {A0, A1, A2, A3, A4};
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL;

String commandBuffer;

void stopMotors() {
  analogWrite(LEFT_ENABLE, 0);
  analogWrite(RIGHT_ENABLE, 0);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
}

void reportSensors() {
  String response = "SENSORS";
  for (int index = 0; index < 5; index++) {
    response += " IR";
    response += index;
    response += "=";
    response += analogRead(IR_SENSOR_PINS[index]);
  }

  digitalWrite(ULTRASONIC_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIGGER, LOW);

  const unsigned long duration =
      pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  response += " RANGE_CM=";
  if (duration == 0) {
    response += "NO_ECHO";
  } else {
    response += String(duration * 0.0343 / 2.0, 1);
  }
  Serial.println(response);
}

void setup() {
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(ULTRASONIC_TRIGGER, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  stopMotors();
  digitalWrite(ULTRASONIC_TRIGGER, LOW);
  Serial.begin(BAUD_RATE);
  Serial.println("READY");
}

void handleCommand(const String& command) {
  if (command == "STOP") {
    stopMotors();
    Serial.println("OK STOPPED");
  } else if (command == "STATUS") {
    Serial.println("STATUS READY MOTION_DISABLED SENSORS_AVAILABLE");
  } else if (command == "SENSORS") {
    reportSensors();
  } else if (command == "START" || command == "FORWARD" ||
             command == "BACKWARD" || command == "LEFT" ||
             command == "RIGHT") {
    stopMotors();
    Serial.println("ERR MOTION_DISABLED");
  } else if (command.length() > 0) {
    stopMotors();
    Serial.println("ERR UNKNOWN_COMMAND");
  }
}

void loop() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n') {
      commandBuffer.trim();
      handleCommand(commandBuffer);
      commandBuffer = "";
    } else if (incoming != '\r') {
      if (commandBuffer.length() < 32) {
        commandBuffer += incoming;
      } else {
        commandBuffer = "";
        stopMotors();
        Serial.println("ERR COMMAND_TOO_LONG");
      }
    }
  }
}
