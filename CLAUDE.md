# champ_jazzy_ws — notas de referencia del proyecto

Workspace ROS 2 Jazzy para un robot cuadrúpedo (basado en Spot Micro,
paquete `robot_dog`) controlado con el framework CHAMP, con un puente
en desarrollo hacia hardware real (ESP32 + PCA9685 + 12 servos DS3235).

## Medidas del URDF vs robot físico (verificado con calibrador)

Archivo: `src/robot_dog/urdf/spot_micro.urdf.xacro`

| Propiedad xacro | Valor URDF | Medido físicamente | Estado |
|---|---|---|---|
| `shift` (coxa→fémur) | 0.055 m (original) | 0.043 m | **Corregido** en el xacro a 0.043 |
| `leg_length` (fémur→rodilla) | 0.1075 m | ~0.110 m | Coincide, sin cambios |
| `foot_length` (rodilla→pie) | 0.130 m | ~0.127 m | Coincide, sin cambios |
| `toe_radius` | 0.020 m | 0.020 m | Coincide exacto |
| `shiftx` | 0.093 m | ~0.090 m | Coincide, sin cambios |
| `shifty` | 0.039 m | ~0.038 m | Coincide, sin cambios |

## Tópico de comandos de CHAMP (confirmado en vivo)

- **Tópico**: `/all_legs_trajectory_controller/joint_trajectory`
- **Tipo de mensaje**: `trajectory_msgs/msg/JointTrajectory` (siempre 1 punto por mensaje; `velocities`/`accelerations`/`effort` vacíos, solo posición)
- **Orden real de publicación de los 12 joints** (verificado con `ros2 topic echo`, coincide con la declaración en `dog_controllers.yaml` y el xacro):
  `front_left_{shoulder,leg,foot}` → `front_right_{...}` → `rear_left_{...}` → `rear_right_{...}`
- **Ejes de rotación**: `shoulder`=eje X, `leg`=eje Y, `foot`=eje Y — idénticos en las 4 patas (sin inversión de signo entre lado izquierdo/derecho; la asimetría L/R vive en la posición de montaje, no en el eje).
- **Límites angulares (rad)**, idénticos en las 4 patas:
  - `shoulder`: `[-0.548, 0.548]`
  - `leg`: `[-2.666, 1.548]`
  - `foot`: `[-2.6, 0.1]`

## Calibración de `ticks_at_zero` (fémur y tibia) — lección importante

CHAMP define matemáticamente "0 rad" para `leg` (fémur) y `foot` (tibia)
según la geometría del URDF, no según ninguna herramienta física de
calibración:
- `leg` (fémur) = 0 rad → fémur colgando **verticalmente hacia abajo**
  (alineado con el eje de la cadera).
- `foot` (tibia) = 0 rad → tibia **en línea recta** con el fémur (sin
  ningún doblez), sin importar el ángulo del fémur — el `rpy="0 0 0"`
  en el `<origin>` de la articulación `foot` en el xacro confirma que
  no hay ninguna rotación de offset entre ambas a 0 rad.

Calibrar `ticks_at_zero` usando una herramienta física de referencia
(por ejemplo, un jig impreso en 3D que fija la rodilla en 45°) da un
valor de "Zero" que **no coincide** con la convención de CHAMP, y
produce un desfase constante en todo el mapeo ángulo→tick de esa
articulación — no solo en la postura de pie. Síntoma típico: al forzar
`nominal_height` para intentar lograr una postura física deseada, hay
que acercarse peligrosamente al límite de alcance de la pata
(`leg_length + foot_length`) sin nunca lograrlo limpiamente.

Procedimiento correcto para recalibrar `ticks_at_zero` de fémur y
tibia de una pata (con el sketch de calibración manual, canal+tick
en ticks crudos del PCA9685, no microsegundos):
1. Mueve fémur y tibia (dos canales, cualquier combinación) hasta que
   la pata quede **completamente recta** (fémur y tibia en línea),
   sin importar hacia dónde apunte — eso da el tick correcto de
   `ticks_at_zero` de la **tibia**, independiente del ángulo del
   fémur.
2. Con la tibia fija en ese tick, mueve solo el fémur hasta que la
   pata (ya recta) cuelgue **verticalmente hacia abajo** — eso da el
   tick correcto de `ticks_at_zero` del **fémur**.
3. `ticks_at_min_rad`/`ticks_at_max_rad` no se tocan (esos sí se
   verificaron correctamente con la comparación de dirección contra
   RViz/simulación).

Una vez recalibrado así, `nominal_height` en `gait.yaml` se puede
ajustar con margen de seguridad normal (lejos del límite físico) para
lograr la postura de pie deseada, porque el mapeo completo queda
consistente con la convención de CHAMP.

## Convención de signos de los ángulos (verificada numéricamente, 25 sept)

Con la regla de la mano derecha sobre los ejes del URDF (`shoulder` = +X,
`leg` y `foot` = +Y), y con X hacia adelante, Y hacia la izquierda, Z arriba:
- `q2` > 0 lleva el fémur (y el pie) **hacia atrás** (-X). La postura de pie de
  CHAMP a 0.13 m es q2 = +1.1445 (fémur hacia atrás-abajo) y q3 = -1.997 (tibia
  hacia adelante-abajo): las rodillas apuntan hacia atrás (`knee_orientation ">>"`).
- `q3` se mide desde la prolongación del fémur; es negativo con la rodilla flexionada.
- `q1` > 0 lleva el pie hacia +Y.
- Cinemática directa: x = -L2 sin q2 - L3 sin(q2+q3); y = s L1 cos q1 + R sin q1;
  z = s L1 sin q1 - R cos q1, con R = L2 cos q2 + L3 cos(q2+q3) y s = +1 izquierda,
  -1 derecha. La inversa (Capítulo 4 de la tesis) coincide con `kinematics.h` de CHAMP
  a 1e-15 rad (4000 objetivos por pata). Script de verificación:
  `tesis_emilio_valenzuela/imagenes/fuentes_figuras/verificacion_cinematica.py`.
- **No** sumar offsets a q2 "a ojo" para mover un pie: el signo es contraintuitivo
  (q2 mayor = pie más atrás). `hardware/scripts_pruebas/creep_una_pata.py` calcula los
  ángulos con la cinemática inversa de CHAMP a partir de posiciones del pie en metros.
  La versión anterior (offsets de q2 y q1 con el signo contrario) caminaba hacia
  atrás y desplazaba el peso hacia la pata levantada.
- Estabilidad estática: con las 4 patas en el rectángulo de apoyo (pies a x=±0.093,
  y=±0.082 m del centro) el centro del cuerpo queda a 0.0 mm del borde del triángulo al
  levantar una pata; desplazarlo al centroide da 41 mm de margen.

## IMU Hiwonder IM10A (USB)

- Conectada por USB (chip adaptador CH340, mismo tipo que usan
  algunas placas ESP32 — verificar con `lsusb`/`udevadm info` cuál
  puerto es cuál si hay más de un dispositivo serie conectado a la
  vez).
- **Baudrate: 9600.** Protocolo estándar Hiwonder/WitMotion (igual al
  usado en sus módulos WT901/JY901): paquetes de 11 bytes, byte 0 =
  `0x55`, byte 1 = tipo de dato, bytes 2-9 = 4 valores `int16` little
  endian, byte 10 = checksum (suma de bytes 0-9 & 0xFF).
  - `0x51` = Aceleración (ax, ay, az, temp) — escala `raw/32768*16` (g)
  - `0x52` = Giroscopio (wx, wy, wz, temp) — escala `raw/32768*2000` (°/s)
  - `0x53` = Ángulo (roll, pitch, yaw, temp) — escala `raw/32768*180` (°)
  - `0x54` = Campo magnético (mx, my, mz, temp)
  - `0x56` = Presión/altitud (sensor barométrico, el "10º eje")
- Verificado leyendo datos reales del puerto y decodificando: valores
  de roll/pitch coherentes con la IMU apoyada plana (~0°), y cambian
  en tiempo real al mover/inclinar el sensor.
- Aún no integrada a ROS2 (no existe todavía un nodo publicando
  `sensor_msgs/Imu`); pendiente si se necesita para `robot_localization`
  o el filtro de estado de CHAMP.

## LiDAR YDLIDAR T-mini Plus (rebrand Yahboom/EAI, TOF 360°, 12m)

- Conectado por USB (adaptador Silicon Labs CP210x — distinto chip al
  CH340 del ESP32/IMU, así que ambos pueden distinguirse por
  `lsusb`/`udevadm info` si están conectados a la vez).
- **Puerto/baudrate confirmados**: `/dev/ttyUSB0` @ `230400`. Modelo
  confirmado por el propio dispositivo al conectar: `Tmini Plus`,
  firmware 1.2.
- SDK: `~/YDLidar-SDK` (clonado de `YDLIDAR/YDLidar-SDK`, compilado con
  cmake/make e instalado con `sudo make install` en `/usr/local`). El
  SDK trae binarios de prueba ya compilados en `build/` (ej.
  `tmini_test`) útiles para verificar el hardware sin pasar por ROS2.
- Driver ROS2: `~/ydlidar_ws/src/ydlidar_ros2_driver` (clonado de
  `YDLIDAR/ydlidar_ros2_driver`, rama `humble` — no existe rama
  `jazzy`, pero compila y funciona bien en Jazzy sin cambios).
- **Archivo de parámetros correcto**: `params/Tmini-Plus-SH.yaml` (no
  `TminiPro.yaml`, que es de otro modelo de la misma familia con
  `lidar_type` distinto). Lanzar con:
  ```
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=<ruta a Tmini-Plus-SH.yaml>
  ```
  Publica `/scan` (`sensor_msgs/LaserScan`) establemente a 10Hz. El
  launch también levanta un `static_transform_publisher` fijo
  `base_link → laser_frame`.
- **Gotcha de QoS con RViz**: el nodo publica `/scan` con
  `Reliability: BEST_EFFORT`. RViz agrega el display `LaserScan` por
  defecto con `Reliability: RELIABLE` — políticas incompatibles, así
  que RViz se suscribe pero nunca recibe nada, sin ningún error
  visible. Hay que cambiar manualmente "Reliability Policy" a
  `Best Effort` en las propiedades del display `LaserScan` para que
  aparezcan los puntos.
- Aún no integrado al resto del stack (Nav2/SLAM); pendiente si se
  necesita para eso.

## Hardware ESP32 (actualizado 25 sept) — 1 placa PCA9685

Desde el 25 sept el robot usa **una sola placa PCA9685** con los 12
servos (hasta el 24 sept eran 2 placas: `0x41` delanteras y `0x40`
traseras, cada una con su buck de 6V).

- El sketch detecta solo la dirección I2C de la placa al arrancar
  (`detectPca()`: prueba `0x41` y luego `0x40`, bus `Wire.begin(21,22)`).
  Si no responde ninguna, el LED parpadea rápido y el firmware se
  detiene (no hay serie de depuración: el puerto USB lo usa micro-ROS).
- Canal de cada joint (`JOINT_TO_CHANNEL[]` en el sketch):

| Pata | shoulder (Coax) | leg (Fémur) | foot (Tibia) |
|---|---|---|---|
| front_left | 13 | 14 | 15 |
| front_right | 0 | 1 | 2 |
| rear_left | 11 | 10 | 9 |
| rear_right | 7 | 6 | 5 |

### Tabla de calibración (25 sept, placa única)

| Joint | Canal | min | zero | max | Pie -> tick |
|---|---|---|---|---|---|
| front_left_shoulder | 13 | 470 | 260 | 80 | |
| front_left_leg | 14 | 290 | 455 | 570 | 525 |
| front_left_foot | 15 | 164 | 470 | 620 | 250 |
| front_right_shoulder | 0 | 90 | 290 | 450 | |
| front_right_leg | 1 | 655 | 615 | 494 | 540 |
| front_right_foot | 2 | 524 | 315 | 190 | 460 |
| rear_left_shoulder | 11 | 580 | 440 | 300 | |
| rear_left_leg | 10 | 295 | 450 | 571 | 525 |
| rear_left_foot | 9 | 155 | 455 | 620 | 240 |
| rear_right_shoulder | 7 | 180 | 310 | 440 | |
| rear_right_leg | 6 | 635 | 595 | 467 | 515 |
| rear_right_foot | 5 | 505 | 290 | 165 | 440 |

**Cómo se construyó (regla a seguir en futuras recalibraciones):**
el usuario da DOS cosas por separado y no hay que mezclarlas:
1. **"Vertical hacia abajo"** (pata recta y colgando): es el
   `ticks_at_zero` de fémur y tibia (el "0 rad" de CHAMP).
2. **Su tabla min/zero/max**: su "Zero" es la **postura de pie** que él
   quiere (NO el vertical). Para el hombro sí coincide con el zero
   porque de pie el hombro vale 0 rad.

Con eso, `ticks_at_max_rad` del fémur (y `ticks_at_min_rad` de la
tibia) se calculan para que el ángulo de pie de CHAMP a
`nominal_height=0.13` (`gait_real.yaml`: fémur +1.1445 rad, tibia
-1.997 rad) caiga exactamente en ese "Zero" de pie, replicando
`angleToTicks()` (margen de seguridad 20 ticks). El sentido (qué
extremo va en min y cuál en max) sale del signo de (pie - vertical) y
coincidió con el de todas las calibraciones ya validadas. El extremo
del lado contrario casi no se usa (fémur negativo / tibia positiva); en
`front_right_leg` y `rear_right_leg` no había dato de ese lado y se
puso a 40 ticks pasada la vertical. Los min/max de los hombros son los
medidos por el usuario (izquierda/derecha en espejo).

**Errores de dirección que ya ocurrieron** (para no repetirlos): dar
como min/max el orden físico "natural" del barrido hace que la pata se
vaya hacia el lado contrario al pedir la postura de pie; se arregla
intercambiando min y max, sin tocar el zero. Y cuando el usuario pega
un bloque de calibración completo, puede traer filas viejas ya
descartadas mezcladas con filas nuevas: comparar contra lo vigente
antes de aplicar (coincidencias dígito por dígito con valores viejos
= copia, no remedición).

## Raspberry Pi 5 (computadora a bordo)

- Ubuntu 24.04.5 LTS (Noble) arm64, IP en la red WiFi:
  `192.168.100.199`, usuario `emilio`, hostname `imhesh`.
- Acceso por SSH con llave (sin contraseña) desde esta PC — llave
  publica en `~/.ssh/id_ed25519.pub` de esta PC, agregada a
  `~/.ssh/authorized_keys` de la Raspberry.
- `sudo` sin contraseña habilitado para el usuario `emilio`
  (`/etc/sudoers.d/emilio-nopasswd`) — decisión explicita del usuario
  para permitir automatizacion remota; baja la seguridad si alguien
  mas accede a la sesion de esta PC.
- Todo instalado y compilado igual que en esta PC (ver secciones de
  arriba para detalles de cada uno):
  - ROS 2 Jazzy Desktop + `ros-dev-tools`
  - `~/champ_jazzy_ws` (transferido con `rsync`, no clonado de git
    remoto — no hay remoto configurado todavia) compilado completo
  - `~/microros_ws` con `micro_ros_agent` compilado
  - `python3-serial` (pyserial) para `imu_publisher.py`
  - `~/YDLidar-SDK` compilado e instalado en `/usr/local`
  - `~/ydlidar_ws` con `ydlidar_ros2_driver` (rama `humble`, igual que
    en esta PC — la rama `master` falla al compilar en Jazzy por una
    API de `declare_parameter` incompatible)
- Pendiente: mantener el workspace sincronizado cuando se hagan
  cambios en esta PC (no hay git remoto ni proceso automatico de
  sync todavia).
- **`arduino-cli` + core ESP32 + todas las librerias** (`Adafruit_BusIO`,
  `Adafruit_PWM_Servo_Driver_Library`, `ESP32Servo`, `micro_ros_arduino`)
  tambien instalados — la carpeta `~/Arduino` completa (sketches +
  librerias) se copio con `rsync` desde esta PC. El core ESP32 SI se
  instalo nativo en la Pi (`arduino-cli core install esp32:esp32`,
  necesario porque el toolchain del compilador es especifico de la
  arquitectura del HOST — ARM64 en la Pi vs x86_64 en esta PC — pero
  las librerias en si son portables porque compilan para el ESP32
  target, no para el host). Verificado compilando
  `esp32_quadruped_bridge.ino` (con `micro_ros_arduino`) ahi mismo,
  sin errores. La Raspberry Pi ya puede reprogramar el ESP32 sin
  depender de esta PC para nada. Agregar `~/bin` al PATH
  (`export PATH=$PATH:$HOME/bin`, ya en `.bashrc`) si `arduino-cli`
  no se encuentra en una sesion nueva.
- Stack de SLAM/Nav2 tambien instalado: `slam-toolbox`,
  `navigation2`, `nav2-bringup` (para los pasos 3-4 del plan de
  navegacion autonoma real).
- Tambien instalado: `ros-gz-sim`, `ros-gz-bridge`, `ros2-control`,
  `ros2-controllers`, `gz-ros2-control`, `joint-state-publisher-gui`,
  `robot-localization` — necesarios para `champ_gazebo.launch.py`.
- **La GPU de la Raspberry Pi 5 (VideoCore VII) NO soporta OpenGL 3.3**,
  requerido por el motor de render Ogre2 de Gazebo Harmonic — asi que
  `champ_gazebo.launch.py` **no se puede usar en la Raspberry Pi**
  (crashea con segfault incluso en modo headless/`-s`, porque el URDF
  tiene sensores `gpu_lidar`/`camera` simulados que fuerzan la
  inicializacion del render aunque no haya ventana). Probado sin
  exito: `LIBGL_ALWAYS_SOFTWARE=1`, `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`,
  `GALLIUM_DRIVER=llvmpipe` — Ogre2/EGL selecciona el dispositivo de
  hardware explicitamente en el codigo, sin caso de fallback a
  software. Esto no importa para el robot real: Gazebo es solo para
  simulacion, y no hace falta para correr el robot fisico.
- **`champ_real.launch.py`** (nuevo, agregado 17 sept, no reemplaza
  nada existente): equivalente a `champ_gazebo.launch.py` pero SIN
  Gazebo — levanta `robot_state_publisher` + `quadruped_controller_node`
  con `use_sim_time: false`, `gazebo: false`, `publish_joint_states: true`
  (nadie mas publica `/joint_states` sin el `joint_state_broadcaster`
  de Gazebo). Publica `/all_legs_trajectory_controller/joint_trajectory`
  a 200Hz estable, verificado funcionando en la Raspberry Pi. Es la
  base para correr el robot real (con el puente ESP32 y luego Nav2/SLAM
  reales) sin necesitar Gazebo en absoluto.
- **Pendiente**: adaptar `slam.launch.py`/`amcl.launch.py`/
  `navigation.launch.py` (hoy hardcodeados a `use_sim_time: True` y a
  lanzar Gazebo ellos mismos) para una variante de hardware real que
  use `champ_real.launch.py` + el LiDAR real (`ydlidar_ros2_driver`)
  en vez de los bridges simulados de Gazebo. Sin tocar los archivos
  existentes (esos siguen siendo para simulacion).

## Puente ESP32 (micro-ROS)

- `micro_ros_arduino` (rama `jazzy`) **sí soporta `trajectory_msgs/JointTrajectory` de fábrica** — verificado descargando el repo real e inspeccionando `available_ros2_types`, y compilando un sketch de prueba con ese tipo (compiló sin errores). Una búsqueda anterior por la API de GitHub había dado un resultado incompleto/erróneo sobre esto — no confiar en esa vía para verificar tipos disponibles, mejor descargar el repo real.
- Aun así, el puente usa `std_msgs/Float32MultiArray` (12 floats en radianes, mismo orden fijo de arriba) en vez de `JointTrajectory` directo, por simplicidad de manejo en el microcontrolador (sin arrays anidados ni strings, asignación estática trivial) — no por limitación de la librería.
- Nodo puente ROS 2: `src/robot_dog/scripts/joint_trajectory_bridge.py` (`/all_legs_trajectory_controller/joint_trajectory` → `/esp32/joint_targets`).
- Sketch ESP32: `~/Arduino/esp32_quadruped_bridge/esp32_quadruped_bridge.ino` — incluye el mapeo real de canales del PCA9685 y la calibración de ticks (0-4095 @ 50Hz, no microsegundos) de los 12 servos, ya cargada.
- Agente: compilado desde fuente en `~/microros_ws/` (no viene por apt para Jazzy).
