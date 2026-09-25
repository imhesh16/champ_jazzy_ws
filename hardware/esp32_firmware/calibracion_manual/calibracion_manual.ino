#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// UN SOLO PCA9685
#define PWM_ADDR 0x41

Adafruit_PWMServoDriver PWM = Adafruit_PWMServoDriver(PWM_ADDR);

const int PULSE_MIN = 30;
const int PULSE_MAX = 700;

void setup() {

  Serial.begin(115200);

  // ESP32 I2C
  Wire.begin(21, 22);  // SDA = 21, SCL = 22

  // PCA9685
  PWM.begin();
  PWM.setPWMFreq(50);

  delay(1000);

  Serial.println();
  Serial.println("=== CALIBRACION PCA9685 ===");
  Serial.println("PCA9685: 0x41");
  Serial.println("Canales habilitados: CH0 a CH15");
  Serial.println();
  Serial.println("NO se mueve ningun servo al iniciar.");
  Serial.println();
  Serial.println("Formato: canal pulso");
  Serial.println("Ejemplo: 14 300");
  Serial.println();
  Serial.println("Rango permitido: 30 a 700");
}

void loop() {

  if (Serial.available()) {

    String entrada = Serial.readStringUntil('\n');
    entrada.trim();

    int espacio = entrada.indexOf(' ');

    if (espacio == -1) {
      Serial.println("Formato incorrecto.");
      Serial.println("Ejemplo: 14 300");
      return;
    }

    int canal = entrada.substring(0, espacio).toInt();
    int pulse = entrada.substring(espacio + 1).toInt();

    // Verificar canal
    if (canal < 0 || canal > 15) {
      Serial.println("Canal invalido.");
      Serial.println("Usa CH0 a CH15.");
      return;
    }

    // Verificar pulso
    if (pulse < PULSE_MIN || pulse > PULSE_MAX) {
      Serial.println("Pulso fuera de rango.");
      Serial.println("Usa valores entre 30 y 700.");
      return;
    }

    // Mover servo
    PWM.setPWM(canal, 0, pulse);

    Serial.print("CH");
    Serial.print(canal);
    Serial.print(" -> ");
    Serial.println(pulse);
  }
}