#include <SoftwareSerial.h>
#include <Servo.h>

// Pin untuk motor servo
Servo myservo; // servo name
SoftwareSerial BT(2, 3); // RX, TX untuk Bluetooth

// Pin untuk motor dan sensor ultrasonik
#define enA 10
#define in1 7  // Pin in1 diubah menjadi 7
#define in2 6  // Pin in2 diubah menjadi 6
#define in3 5  // Pin in3 diubah menjadi 5
#define in4 4  // Pin in4 diubah menjadi 4
#define enB 9
#define trigPin 12 // Pin Trig HC-SR04
#define echoPin 11 // Pin Echo HC-SR04

int xAxis = 140, yAxis = 140;
int motorSpeedA = 0;
int motorSpeedB = 0;

long duration; // Durasi sinyal ultrasonik
int distance;  // Jarak yang diukur dalam cm

void setup() {
  // Setup untuk motor
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trigPin, OUTPUT); // Trig sebagai output
  pinMode(echoPin, INPUT);  // Echo sebagai input

  // Setup untuk komunikasi serial
  Serial.begin(9600);
  BT.begin(9600); // Setup komunikasi Bluetooth
  delay(500);

  // Setup servo
  myservo.attach(13); // Servo dihubungkan ke pin 13
}

void loop() {
  // Membaca jarak dari sensor ultrasonik
  distance = readUltrasonic();

  // Menampilkan jarak di Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Membaca data joystick dari Bluetooth
  while (BT.available() >= 2) {
    xAxis = BT.read();
    delay(10);
    yAxis = BT.read();
    Serial.print("Joystick X: ");
    Serial.print(xAxis);
    Serial.print(", Y: ");
    Serial.println(yAxis);
  }

  // Mengontrol servo dengan data dari Bluetooth
  if (BT.available() > 0) {
    int servoPos = BT.read(); // Membaca nilai untuk posisi servo
    Serial.print("Servo position: ");
    Serial.println(servoPos); // Menampilkan posisi servo yang diterima
    myservo.write(servoPos); // Menyesuaikan posisi servo
  }

  // Logika pergerakan motor berdasarkan joystick
  if (xAxis > 130 && xAxis < 150 && yAxis > 130 && yAxis < 150) {
    Stop();
  }

  if (yAxis > 130 && yAxis < 150) {
    if (xAxis < 130) {
      turnRight();
      motorSpeedA = map(xAxis, 130, 60, 0, 255);
      motorSpeedB = map(xAxis, 130, 60, 0, 255);
    }

    if (xAxis > 150) {
      turnLeft();
      motorSpeedA = map(xAxis, 150, 220, 0, 255);
      motorSpeedB = map(xAxis, 150, 220, 0, 255);
    }
  } else {
    if (xAxis > 130 && xAxis < 150) {
      if (yAxis < 130) {
        forword();
        motorSpeedA = map(yAxis, 130, 60, 0, 255);
        motorSpeedB = map(yAxis, 130, 60, 0, 255);
      }
      if (yAxis > 150) {
        backword();
        motorSpeedA = map(yAxis, 150, 220, 0, 255);
        motorSpeedB = map(yAxis, 150, 220, 0, 255);
      }
    } else {
      if (yAxis < 130) {
        forword();
      }
      if (yAxis > 150) {
        backword();
      }

      if (xAxis < 130) {
        motorSpeedA = map(xAxis, 130, 60, 255, 50);
        motorSpeedB = 255;
      }

      if (xAxis > 150) {
        motorSpeedA = 255;
        motorSpeedB = map(xAxis, 150, 220, 255, 50);
      }
    }
  }

  analogWrite(enA, motorSpeedA); // Kirim sinyal PWM ke motor A
  analogWrite(enB, motorSpeedB); // Kirim sinyal PWM ke motor B
}

// Fungsi untuk membaca jarak dari sensor ultrasonik
int readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH); // Baca durasi sinyal Echo
  distance = duration * 0.034 / 2;  // Hitung jarak dalam cm
  return distance;
}

// Fungsi kontrol motor
void forword() {
  Serial.println("forword");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void backword() {
  Serial.println("backword");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnRight() {
  Serial.println("turnRight");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnLeft() {
  Serial.println("turnLeft");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  Serial.println("stop");
}
