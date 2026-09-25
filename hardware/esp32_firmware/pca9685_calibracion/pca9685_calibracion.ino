// Calibracion manual de pulsos PWM para servos DS3235 via PCA9685.
//
// No usa ROS/micro-ROS -- solo sirve para encontrar, con el servo real
// conectado, que pulso (en microsegundos) corresponde a cada extremo
// mecanico seguro de movimiento, o a una posicion de referencia
// especifica (como "femur y tibia en linea recta").
//
// Version DOS CANALES: controla femur y tibia de la misma pata al
// mismo tiempo, para poder buscar la posicion de pata recta moviendo
// ambos, en vez de dejar uno fijo.
//
// Uso por el Monitor Serie (115200 baudios):
//   - Escribe "f<pulso>" y Enter -> mueve el FEMUR a ese pulso (us).
//     Ejemplo: f1500
//   - Escribe "t<pulso>" y Enter -> mueve la TIBIA a ese pulso (us).
//     Ejemplo: t1400
//   - Ve moviendote de a poco (50 en 50 us) en cualquiera de los dos
//     hasta que la pata quede completamente recta (femur y tibia en
//     linea), o hasta encontrar el limite mecanico seguro que estes
//     buscando en cada caso.
//   - Anota el pulso de CADA canal cuando logres la posicion buscada.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_I2C_ADDR 0x40
#define PCA9685_PWM_FREQ_HZ 50

// TODO: cambia esto a los canales de la pata que estas probando.
#define FEMUR_CHANNEL 5  // front_right_leg (femur)
#define TIBIA_CHANNEL 6  // front_right_foot (tibia)

// Coax (shoulder) de la misma pata: no se calibra aqui, solo se manda
// una vez a su tick de Zero ya validado, para que no quede en una
// posicion residual rara mientras calibramos femur/tibia.
#define COAX_CHANNEL 4
#define COAX_ZERO_TICKS 355  // front_right_shoulder, ya validado

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);

void setServoPulseUs(uint8_t channel, uint16_t pulse_us) {
    uint32_t ticks = ((uint32_t)pulse_us * 4096UL) / 20000UL; // periodo 20000us @ 50Hz
    pwm.setPWM(channel, 0, (uint16_t)ticks);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(PCA9685_PWM_FREQ_HZ);

    Serial.println("=== Calibracion PCA9685 (femur + tibia) ===");
    Serial.print("Femur en canal: ");
    Serial.println(FEMUR_CHANNEL);
    Serial.print("Tibia en canal: ");
    Serial.println(TIBIA_CHANNEL);
    Serial.println("Escribe 'f<pulso>' para mover el femur, 't<pulso>' para la tibia.");
    Serial.println("Ejemplo: f1500  o  t1450");
    Serial.println("Rango seguro: 400 a 2600 us.");

    // Posicion inicial segura: centro en ambos.
    setServoPulseUs(FEMUR_CHANNEL, 1500);
    setServoPulseUs(TIBIA_CHANNEL, 1500);
    Serial.println("Posicion inicial: ambos en 1500us (centro aproximado).");

    // Coax a su Zero ya validado (tick directo, no en microsegundos).
    pwm.setPWM(COAX_CHANNEL, 0, COAX_ZERO_TICKS);
    Serial.println("Coax mandado a su Zero ya validado.");
}

void loop() {
    if (Serial.available() > 0) {
        char prefix = Serial.read();

        if (prefix == 'f' || prefix == 'F' || prefix == 't' || prefix == 'T') {
            long pulse_us = Serial.parseInt();

            // Descarta el salto de linea sobrante.
            while (Serial.available() > 0 && Serial.peek() == '\n') {
                Serial.read();
            }

            if (pulse_us >= 400 && pulse_us <= 2600) {
                uint8_t channel = (prefix == 'f' || prefix == 'F') ? FEMUR_CHANNEL : TIBIA_CHANNEL;
                const char *label = (prefix == 'f' || prefix == 'F') ? "Femur" : "Tibia";

                setServoPulseUs(channel, (uint16_t)pulse_us);
                Serial.print(label);
                Serial.print(" -> ");
                Serial.print(pulse_us);
                Serial.println("us");
            } else {
                Serial.println("Valor fuera de rango seguro (400-2600us). Ignorado.");
            }
        }
        // Cualquier otro caracter (como saltos de linea sueltos) se ignora.
    }
}
