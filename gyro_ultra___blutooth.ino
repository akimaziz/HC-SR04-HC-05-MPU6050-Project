#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>

SoftwareSerial bluetoothSerial(10, 11); // RX, TX pins for Bluetooth communication

const int trigPin = 7;    // Trigger pin of the ultrasonic sensor
const int echoPin = 8;    // Echo pin of the ultrasonic sensor
const int merahLedPin = 12;
const int kuningLedPin = 13;

MPU6050 mpu;

int16_t anchorX = 0; // Anchor values for gyrometer readings
int16_t anchorY = 0;
int16_t anchorZ = 0;

void setup() {
  Serial.begin(9600);  // Serial monitor communication
  Wire.begin(); // Initialize I2C communication
  mpu.initialize();  // Initialize the gyrometer sensor

  Serial.print("MPU6050 initialization: ");
  Serial.println(mpu.testConnection() ? "Successful" : "Failed");

  bluetoothSerial.begin(9600);  // Bluetooth communication
  pinMode(trigPin, OUTPUT);  // Trigger pin as output
  pinMode(echoPin, INPUT);   // Echo pin as input
  pinMode(merahLedPin, OUTPUT);
  pinMode(kuningLedPin, OUTPUT);

  // Calibrate gyrometer by placing the sensor in a stable position for a few seconds
  // Uncomment the following lines during the initial setup and follow the instructions in the Serial Monitor
  Serial.println("Gyrometer Calibration:");
  Serial.println("Keep the sensor stable and don't touch it.");
  delay(2000);
  mpu.CalibrateGyro();
  Serial.println("Calibration complete!");

  // Initialize anchor values
  anchorX = 0;
  anchorY = 0;
  anchorZ = 0;
}

void loop() {
  bluetooth();
  sensor();
  gyrometer();
}

void bluetooth() {
  if (bluetoothSerial.available()) {
    char command = bluetoothSerial.read();
    // Add your Bluetooth command handling logic here
    // For example:
    if (command == '1') {
      // Perform an action when command '1' is received
      Serial.println("Command 1 received");
    } else if (command == '2') {
      // Perform an action when command '2' is received
      Serial.println("Command 2 received");
    }
  }
}

void sensor() {
  long duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // Send a 10us pulse to trigger the ultrasonic sensor
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);  // Measure the pulse duration from the echo pin
  distance = duration * 0.034 / 2;   // Calculate the distance in centimeters

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 5) {
    digitalWrite(merahLedPin, HIGH);
    digitalWrite(kuningLedPin, LOW);
    Serial.println("Jarak kurang dari 5cm");
  } else if (distance > 50) {
    digitalWrite(merahLedPin, LOW);
    digitalWrite(kuningLedPin, HIGH);
    Serial.println("Jarak lebih dari 50cm");
  } else {
    digitalWrite(merahLedPin, LOW);
    digitalWrite(kuningLedPin, LOW);
  }
}

void gyrometer() {
  // Read gyrometer sensor values
  int16_t gyroX = mpu.getRotationX() - anchorX;
  int16_t gyroY = mpu.getRotationY() - anchorY;
  int16_t gyroZ = mpu.getRotationZ() - anchorZ;

  // Adjust gyrometer readings from -360 to 360 degrees
  gyroX = map(gyroX, -32768, 32767, -360, 360);
  gyroY = map(gyroY, -32768, 32767, -360, 360);
  gyroZ = map(gyroZ, -32768, 32767, -360, 360);

  // Print gyrometer readings
  Serial.print("Gyrometer Readings (degrees): ");
  Serial.print("X = ");
  Serial.print(gyroX);
  Serial.print("° | Y = ");
  Serial.print(gyroY);
  Serial.print("° | Z = ");
  Serial.print(gyroZ);
  Serial.println("°");
}
