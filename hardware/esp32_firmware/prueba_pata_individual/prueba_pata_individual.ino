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

// Calibracion FINAL confirmada
Servo servos[12] = {
  {"FL COXA",  12, 620, 400, 200},
  {"FL FEMUR", 13, 505, 420, 180},
  {"FL TIBIA", 14, 645, 250, 95},

  {"FR COXA",  4,  135, 355, 540},
  {"FR FEMUR", 5,  460, 542, 165},
  {"FR TIBIA", 6,  200, 480, 595},

  {"RL COXA",  8,  460, 290, 140},
  {"RL FEMUR", 9,  255, 170, 548},
  {"RL TIBIA", 10, 650, 250, 90},

  {"RR COXA",  0,  155, 305, 445},
  {"RR FEMUR", 1,  440, 520, 130},
  {"RR TIBIA", 2,  95,  365, 485}
};

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

void irATodosZero() {
  for (int i = 0; i < 12; i++) {
    PWM.setPWM(servos[i].canal, 0, servos[i].zero);
  }
}

void probarPata(int idx, const char* nombre) {
  Serial.print("===== PATA "); Serial.print(nombre); Serial.println(" =====");
  for (int j = 0; j < 3; j++) {
    probarServo(servos[idx + j]);
  }
  Serial.println("===== Pata completa. De vuelta en Zero. =====");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  PWM.begin();
  PWM.setPWMFreq(50);
  delay(1000);

  Serial.println("Llevando las 12 a Zero...");
  irATodosZero();
  delay(1500);

  Serial.println();
  Serial.println("=== PRUEBA DE UNA SOLA PATA ===");
  Serial.println("Escribe un numero y Enter en el Monitor Serial:");
  Serial.println("  1 = DELANTERA IZQUIERDA");
  Serial.println("  2 = DELANTERA DERECHA");
  Serial.println("  3 = TRASERA IZQUIERDA");
  Serial.println("  4 = TRASERA DERECHA");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': probarPata(0, "DELANTERA IZQUIERDA"); break;
      case '2': probarPata(3, "DELANTERA DERECHA");   break;
      case '3': probarPata(6, "TRASERA IZQUIERDA");   break;
      case '4': probarPata(9, "TRASERA DERECHA");     break;
    }
  }
}
