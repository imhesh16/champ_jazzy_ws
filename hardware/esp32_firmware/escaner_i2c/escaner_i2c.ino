#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void escanearI2C() {

  Serial.println();
  Serial.println("===========================");
  Serial.println("     ESCANEANDO I2C");
  Serial.println("===========================");

  int encontrados = 0;

  for (uint8_t address = 1; address < 127; address++) {

    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {

      Serial.print("Encontrado: 0x");

      if (address < 16) {
        Serial.print("0");
      }

      Serial.println(address, HEX);
      encontrados++;
    }
  }

  Serial.println("---------------------------");

  if (encontrados == 0) {
    Serial.println("❌ No se encontro ningun dispositivo I2C");
  } else {
    Serial.print("Total encontrados: ");
    Serial.println(encontrados);
  }

  Serial.println("===========================");
  Serial.println("Escribe 'e' + Enter para escanear nuevamente.");
  Serial.println();
}


void setup() {

  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("=== TEST PCA9685 ===");
  Serial.println("e = escanear I2C");

  // Primer escaneo automatico
  escanearI2C();
}


void loop() {

  if (Serial.available()) {

    char comando = Serial.read();

    // Limpiar el resto de caracteres del buffer
    while (Serial.available()) {
      Serial.read();
    }

    if (comando == 'e' || comando == 'E') {
      escanearI2C();
    }
  }
}
