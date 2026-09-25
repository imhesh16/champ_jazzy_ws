// Prueba AISLADA de la pata delantera izquierda unicamente.
//
// Seguridad: los 9 canales de las otras 3 patas quedan SIN NINGUNA
// senal PWM (setPWM(canal, 0, 0) = 0% de duty cycle, no hay pulso).
// Un servo sin pulso no recibe orden de movimiento ni de sujecion --
// no se les manda "quedate quieto en Zero", se les corta la senal
// por completo. Solo los 3 canales de la pata delantera izquierda
// (Coax=12, Femur=13, Tibia=14) reciben comandos, y nunca se tocan
// los otros 9 canales en ningun punto del programa.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PWM_SERVO_ADDR 0x40
Adafruit_PWMServoDriver PWM = Adafruit_PWMServoDriver(PWM_SERVO_ADDR);

struct Servo {
  const char* nombre;
  int canal;
  int min;
  int zero;
  int max;
};

// Solo la pata delantera izquierda. Calibracion FINAL confirmada.
Servo pataFrontLeft[3] = {
  {"FL COXA",  12, 620, 400, 200},
  {"FL FEMUR", 13, 505, 420, 180},
  {"FL TIBIA", 14, 645, 250, 95},
};

// Los 9 canales del resto de las patas: quedan explicitamente
// apagados (sin PWM) y NUNCA se vuelven a tocar en el resto del codigo.
const int CANALES_DESACTIVADOS[9] = {0, 1, 2, 4, 5, 6, 8, 9, 10};

int spd = 15;
int pausaEntreServos = 2000;
const int MARGEN = 20;

int aplicarMargen(int desde, int hacia) {
  return (hacia > desde) ? (hacia - MARGEN) : (hacia + MARGEN);
}

void moverGradual(int canal, int desde, int hasta) {
  if (desde == hasta) return;
  if (desde < hasta) {
    for (int x = desde; x <= hasta; x++) { PWM.setPWM(canal, 0, x); delay(spd); }
  } else {
    for (int x = desde; x >= hasta; x--) { PWM.setPWM(canal, 0, x); delay(spd); }
  }
}

void probarServo(Servo s) {
  int minSeg = aplicarMargen(s.zero, s.min);
  int maxSeg = aplicarMargen(s.zero, s.max);

  Serial.print("--- "); Serial.print(s.nombre);
  Serial.print(" (CH"); Serial.print(s.canal); Serial.println(")");

  Serial.print("  Zero -> Max ("); Serial.print(maxSeg); Serial.println(")");
  moverGradual(s.canal, s.zero, maxSeg);
  delay(pausaEntreServos);

  Serial.print("  Max -> Min ("); Serial.print(minSeg); Serial.println(")");
  moverGradual(s.canal, maxSeg, minSeg);
  delay(pausaEntreServos);

  Serial.println("  Min -> Zero");
  moverGradual(s.canal, minSeg, s.zero);
  delay(pausaEntreServos);
}

void desactivarOtrasPatas() {
  Serial.println("Desactivando (0% PWM) los 9 canales de las otras 3 patas...");
  for (int i = 0; i < 9; i++) {
    PWM.setPWM(CANALES_DESACTIVADOS[i], 0, 0);
  }
  Serial.println("Listo. Esos 9 canales no reciben ninguna senal.");
}

void irFrontLeftAZero() {
  for (int i = 0; i < 3; i++) {
    PWM.setPWM(pataFrontLeft[i].canal, 0, pataFrontLeft[i].zero);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  PWM.begin();
  PWM.setPWMFreq(50);
  delay(1000);

  desactivarOtrasPatas();

  Serial.println("Llevando SOLO la pata delantera izquierda a Zero...");
  irFrontLeftAZero();
  delay(1500);

  Serial.println();
  Serial.println("=== PRUEBA AISLADA: SOLO PATA DELANTERA IZQUIERDA ===");
  Serial.println("Escribe '1' y Enter en el Monitor Serial para probarla.");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      Serial.println("===== PATA DELANTERA IZQUIERDA =====");
      for (int j = 0; j < 3; j++) {
        probarServo(pataFrontLeft[j]);
      }
      Serial.println("===== Completo. De vuelta en Zero. =====");
    }
  }
}
