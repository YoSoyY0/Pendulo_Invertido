// Librerias 12C para controlar el apu6050
// la libreria MPU6050.h necesita 12Cdev., 12Cdev. necesita Wire.h #include "12cdev.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
// La direccia del MPU6050 puede ser 0x68 0x69, dependiendo
// del estado de ADO. Si no se especifica, 0x68 estar implicito
MPU6050 sensor;

// Pines motores
const int pinPWMA = 5;
const int pinAIN2 = 2;
const int pinAIN1 = 0;
const int pinPWMB = 17;
const int pinBIN1 = 4;
const int pinBIN2 = 16;
const int pinSTBY = 15;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z int ax, ay, az
int16_t ax, ay, az;
int16_t gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x = 0.0, ang_y = 0.0;
float ang_x_prev = 0.0, ang_y_prev = 0.0;

// PID
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float integral = 0.0;
float derivativo = 0.0;
float error_previo = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.initialize();

  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinSTBY, OUTPUT);

  tiempo_prev = millis();

}

void loop() {
  // Iniciar MPU
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  // Conseguir angulos
  float accel_ang_x;
  accel_ang_x = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI);
  // manejar tiempo
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  // Aumentar precision del angulo del eje x
  ang_x = 0.97 * (ang_x_prev + (gx / 131.0) * dt) + 0.03 * accel_ang_x; //Con confianza del 97% del giroscopio y del 3% del acelerometro
  ang_x_prev = ang_x;
  // PID
  float desired_angle = 0.0;    //Se quiere que esté recto
  float error = desired_angle - ang_x;    //Error a corregir
  integral += error * dt;                 //Calcular la parte integrativa del PID
  derivativo = (error - error_previo) / dt; //Y la parte derivativa
  float motor_speed = (Kp * error + Ki * integral + Kd * derivativo);   //La velocidad de las ruedas dependerá de el tamaño del error y el pid
  error_previo = error;         //Guardar el error para proximas iteraciones

  // Control de motores
  enableMotors();     
  moveMotor(pinPWMA, pinAIN1, pinAIN2, motor_speed);
  moveMotor(pinPWMB, pinBIN1, pinBIN2, motor_speed);
  // Dar informacion
  Serial.print("Ángulo X: "); Serial.print(ang_x);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Salida PID: "); Serial.print(motor_speed);
  Serial.print(" | Kp: "); Serial.print(Kp);
  Serial.print(" | Ki: "); Serial.print(Ki);
  Serial.print(" | Kd: "); Serial.println(Kd);
  // recibir comandos
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'u': Kp += 1; break;
      case 'j': Kp -= 1; break;
      case 'i': Ki += 0.01; break;
      case 'k': Ki -= 0.01; break;
      case 'o': Kd += 0.1; break;
      case 'l': Kd -= 0.1; break;
    }
    //Interpretar comandos y dar feedback
    Serial.print("Nuevo Kp: "); Serial.print(Kp);
    Serial.print(" | Ki: "); Serial.print(Ki);
    Serial.print(" | Kd: "); Serial.println(Kd);
  }

  delay(5);
}

//Funcion complementaria para manejar el motor
void moveMotor(int pinPWM, int pinIN1, int pinIN2, float speed) {
  if (speed > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWM, speed);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    analogWrite(pinPWM, -speed);
  }
}

void enableMotors() {
  digitalWrite(pinSTBY, HIGH);
}