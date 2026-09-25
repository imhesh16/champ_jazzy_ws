// Puente ESP32 <-> ROS2/CHAMP para el robot cuadrupedo (12 servos DS3235 via 1x PCA9685).
//
// Recibe std_msgs/Float32MultiArray en el topico /esp32/joint_targets
// (publicado por el nodo puente joint_trajectory_bridge.py en la RPi5),
// con 12 angulos en RADIANES, en este orden fijo:
//
//   0 front_left_shoulder   4 front_right_shoulder   8  rear_left_shoulder
//   1 front_left_leg        5 front_right_leg         9  rear_left_leg
//   2 front_left_foot       6 front_right_foot        10 rear_left_foot
//   3 (repite patron por pata)                        11 rear_right_foot
//
// HARDWARE (actualizado 25 sept): UNA sola placa PCA9685 en el bus I2C
// (Wire.begin(21,22)) con los 12 servos. Su direccion se detecta sola
// al arrancar (se prueba 0x41 y luego 0x40); si no responde ninguna,
// el LED parpadea rapido y el firmware se detiene.
// (Hasta el 24 sept eran 2 placas: 0x41 delanteras y 0x40 traseras.)
//
// Transporte micro-ROS: serie (USB), 115200 baudios. Requiere correr
// el micro-ROS agent en la RPi5:
//   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// No todas las variantes de placa ESP32 definen LED_BUILTIN.
// TODO: ajusta el pin al LED real de tu placa si difiere de GPIO2.
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ---------------------------------------------------------------------------
// Configuracion general
// ---------------------------------------------------------------------------

#define NUM_JOINTS 12
#define PCA9685_PWM_FREQ_HZ 50   // estandar para servos analogicos/digitales

// MODO DE PRUEBA: mientras esto sea 'true', solo se energizan y
// comandan las patas dentro del RANGO [TEST_LEG_START, TEST_LEG_END)
// (en orden front_left=0, front_right=1, rear_left=2, rear_right=3).
// El resto de canales quedan sin ninguna senal PWM (0% duty) y se
// ignora cualquier valor que llegue para ellos por ROS2. Cambia a
// 'false' una vez validadas las 4 patas para habilitar las 12
// normalmente.
//
// IMPORTANTE: se reinicia a "solo front_left" tras el cambio de
// hardware del 16 sept (2 placas PCA9685, canales reasignados) --
// las validaciones fisicas anteriores ya no aplican con el nuevo
// mapeo, hay que re-verificar pata por pata.
static const bool TEST_MODE_SINGLE_LEG = true;

// Rango de patas activas ahora mismo (indice de pata, no de joint):
//   front_left=0, front_right=1, rear_left=2, rear_right=3
// Ejemplos: {0,1} = solo front_left. {1,2} = solo front_right.
// {0,2} = front_left + front_right. {0,4} = las 4 (todas).
static const int TEST_LEG_START = 0;
static const int TEST_LEG_END = 4;

// --- Tipos (definidos primero para evitar el problema del Arduino IDE
// generando prototipos automaticos antes de conocer estos structs) ---

struct JointLimits {
    float min_rad;
    float max_rad;
};

struct ServoCalibration {
    uint16_t ticks_at_min_rad;
    uint16_t ticks_at_zero;
    uint16_t ticks_at_max_rad;
};

// Se crea en setup() una vez detectada la direccion I2C de la placa.
Adafruit_PWMServoDriver *pca = nullptr;

// Canal (0-15) de la placa unica para cada uno de los 12 joints, segun
// cableado real (actualizado 25 sept).
static const uint8_t JOINT_TO_CHANNEL[NUM_JOINTS] = {
    13, 14, 15,  // front_left:  shoulder(Coax), leg(Femur), foot(Tibia)
    0,  1,  2,   // front_right: shoulder(Coax), leg(Femur), foot(Tibia)
    11, 10, 9,   // rear_left:   shoulder(Coax), leg(Femur), foot(Tibia)
    7,  6,  5,   // rear_right:  shoulder(Coax), leg(Femur), foot(Tibia)
};

// Limites articulares reales (radianes), tomados del URDF/xacro del robot.
// shoulder: axis X, leg y foot: axis Y (ver Capitulo 4 de la tesis / joints.yaml)
// No cambian con el hardware -- son limites de articulacion, no de cableado.
static const JointLimits LIMITS[NUM_JOINTS] = {
    {-0.548f, 0.548f},   // front_left_shoulder
    {-2.666f, 1.548f},   // front_left_leg
    {-2.6f,   0.1f},     // front_left_foot
    {-0.548f, 0.548f},   // front_right_shoulder
    {-2.666f, 1.548f},   // front_right_leg
    {-2.6f,   0.1f},     // front_right_foot
    {-0.548f, 0.548f},   // rear_left_shoulder
    {-2.666f, 1.548f},   // rear_left_leg
    {-2.6f,   0.1f},     // rear_left_foot
    {-0.548f, 0.548f},   // rear_right_shoulder
    {-2.666f, 1.548f},   // rear_right_leg
    {-2.6f,   0.1f},     // rear_right_foot
};

// Ticks del PCA9685 (0-4095 @ 50Hz) que corresponden a min_rad, 0 rad y
// max_rad de CADA servo.
//
// CALIBRACION NUEVA, PLACA UNICA (25 sept). Reemplaza toda tabla anterior.
//  - ticks_at_zero de leg/foot = la pata recta y colgando vertical
//    (medido a mano; es el "0 rad" de CHAMP).
//  - ticks_at_zero de shoulder = el "Zero" que dio el usuario (para el
//    hombro, la postura de pie usa el mismo angulo 0).
//  - leg: ticks_at_max_rad calculado para que la postura de pie de
//    CHAMP (nominal_height=0.13: fémur +1.1445 rad) caiga en el "Zero"
//    que midio el usuario a mano. foot: idem con ticks_at_min_rad
//    (tibia -1.997 rad). Verificado con un script que replica
//    angleToTicks(): error < 1 tick.
//  - El extremo del lado contrario (leg min / foot max) casi no se usa
//    (angulos negativos de fémur / positivos de tibia); es el tope medido
//    por el usuario. En front_right_leg y rear_right_leg no habia dato en
//    ese lado, asi que se dejo 40 ticks pasada la vertical.
//  - shoulder: min/max tal como los midio el usuario (izq/der en espejo,
//    igual que en las calibraciones anteriores ya validadas).
static const ServoCalibration CALIBRATION[NUM_JOINTS] = {
    {470, 260, 80},    // front_left_shoulder   (CH13)
    {290, 455, 570},   // front_left_leg        (CH14)  parado -> 525
    {164, 470, 620},   // front_left_foot       (CH15)  parado -> 250
    {90,  290, 450},   // front_right_shoulder  (CH0)
    {655, 615, 494},   // front_right_leg       (CH1)   parado -> 540
    {524, 315, 190},   // front_right_foot      (CH2)   parado -> 460
    {580, 440, 300},   // rear_left_shoulder    (CH11)
    {295, 450, 571},   // rear_left_leg         (CH10)  parado -> 525
    {155, 455, 620},   // rear_left_foot        (CH9)   parado -> 240
    {180, 310, 440},   // rear_right_shoulder   (CH7)  recalibrado 25 sept tras aflojarse el hombro
    {635, 595, 467},   // rear_right_leg        (CH6)   parado -> 515
    {505, 290, 165},   // rear_right_foot       (CH5)   parado -> 440
};

// ---------------------------------------------------------------------------
// micro-ROS
// ---------------------------------------------------------------------------

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray joint_targets_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Buffer estatico para el array del mensaje (evita malloc en el callback).
static float joint_targets_buffer[NUM_JOINTS];

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }

void error_loop() {
    while (true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(200);
    }
}

// Convierte un angulo en radianes (dentro de [min_rad, max_rad]) al
// tick de PCA9685 correspondiente, mediante interpolacion lineal EN DOS
// TRAMOS usando el "Zero" real del servo como pivote en angle_rad=0:
//   - angulo en [min_rad, 0]: interpola entre ticks_at_min_rad y ticks_at_zero
//   - angulo en [0, max_rad]: interpola entre ticks_at_zero y ticks_at_max_rad
// Esto evita asumir que el servo es perfectamente lineal/simetrico entre
// sus dos extremos; el pivote en 0 rad siempre cae exactamente en el
// Zero medido, sin importar la forma de la calibracion en cada tramo.
// Margen de seguridad: nunca se alcanza el tick extremo exacto de la
// calibracion, se detiene SAFETY_MARGIN_TICKS antes.
static const float SAFETY_MARGIN_TICKS = 20.0f;

uint16_t angleToTicks(float angle_rad, const JointLimits &limits, const ServoCalibration &cal) {
    float clamped = angle_rad;
    if (clamped < limits.min_rad) clamped = limits.min_rad;
    if (clamped > limits.max_rad) clamped = limits.max_rad;

    // Cada extremo se desplaza 20 ticks hacia el lado del Zero, sin
    // importar si esa pata es creciente o decreciente entre min/zero/max.
    float min_dir = (cal.ticks_at_zero >= cal.ticks_at_min_rad) ? 1.0f : -1.0f;
    float max_dir = (cal.ticks_at_zero >= cal.ticks_at_max_rad) ? 1.0f : -1.0f;
    float safe_min_ticks = (float)cal.ticks_at_min_rad + SAFETY_MARGIN_TICKS * min_dir;
    float safe_max_ticks = (float)cal.ticks_at_max_rad + SAFETY_MARGIN_TICKS * max_dir;

    float ticks;
    if (clamped < 0.0f) {
        float ratio = (clamped - limits.min_rad) / (0.0f - limits.min_rad);
        ticks = safe_min_ticks + ratio * ((float)cal.ticks_at_zero - safe_min_ticks);
    } else {
        float ratio = (clamped - 0.0f) / (limits.max_rad - 0.0f);
        ticks = cal.ticks_at_zero + ratio * (safe_max_ticks - (float)cal.ticks_at_zero);
    }
    return (uint16_t)ticks;
}

// joint_index (0-11): se resuelve al canal correcto de la placa unica.
void setServoTicks(int joint_index, uint16_t ticks) {
    pca->setPWM(JOINT_TO_CHANNEL[joint_index], 0, ticks);
}

void setServoOff(int joint_index) {
    pca->setPWM(JOINT_TO_CHANNEL[joint_index], 0, 0);
}

// Busca la placa PCA9685 conectada (prueba 0x41 y luego 0x40). Si no
// responde ninguna, el LED parpadea rapido y el firmware se detiene.
Adafruit_PWMServoDriver *detectPca() {
    const uint8_t candidates[] = {0x41, 0x40};
    for (uint8_t addr : candidates) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            return new Adafruit_PWMServoDriver(addr);
        }
    }
    error_loop();
    return nullptr;
}

void jointTargetsCallback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size != NUM_JOINTS) {
        // Mensaje con tamano inesperado: se ignora por seguridad.
        return;
    }

    int range_start = TEST_LEG_START * 3;  // inclusivo
    int range_end = TEST_LEG_END * 3;      // exclusivo

    for (int i = 0; i < NUM_JOINTS; i++) {
        // En modo de prueba, solo se obedecen los indices dentro del
        // rango de patas activas; el resto se ignora por completo,
        // sin llamar setServoTicks() para esos canales.
        if (TEST_MODE_SINGLE_LEG && (i < range_start || i >= range_end)) {
            continue;
        }
        float angle_rad = msg->data.data[i];
        uint16_t ticks = angleToTicks(angle_rad, LIMITS[i], CALIBRATION[i]);
        setServoTicks(i, ticks);
    }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Transporte micro-ROS por serie. Cambia a
    // set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);
    // si prefieres WiFi en vez de USB.
    set_microros_transports();

    delay(2000);

    Wire.begin(21, 22);  // SDA = 21, SCL = 22 (ajusta si tu cableado es distinto)

    pca = detectPca();
    pca->begin();
    pca->setPWMFreq(PCA9685_PWM_FREQ_HZ);

    if (TEST_MODE_SINGLE_LEG) {
        int range_start = TEST_LEG_START * 3;
        int range_end = TEST_LEG_END * 3;

        // Apaga por completo (0% PWM, sin senal) los canales fuera del
        // rango activo. El
        // PCA9685 conserva sus valores anteriores aunque se reprograme
        // el ESP32, asi que sin esto podrian quedar en cualquier
        // posicion residual de un sketch anterior.
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (i < range_start || i >= range_end) {
                setServoOff(i);
            }
        }
        // Lleva las patas del rango activo a su Zero conocido, sin
        // esperar el primer mensaje de ROS2.
        for (int i = range_start; i < range_end; i++) {
            setServoTicks(i, CALIBRATION[i].ticks_at_zero);
        }
    }

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_quadruped_bridge", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/esp32/joint_targets"));

    // Reserva estatica del array del mensaje (sin malloc dinamico por mensaje).
    joint_targets_msg.data.data = joint_targets_buffer;
    joint_targets_msg.data.capacity = NUM_JOINTS;
    joint_targets_msg.data.size = 0;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &subscriber, &joint_targets_msg,
        &jointTargetsCallback, ON_NEW_DATA));
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
