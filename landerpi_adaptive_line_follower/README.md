# landerpi_adaptive_line_follower

C++ upgrade de `line_following.py` del stock: sigue una línea de color a
velocidad lineal **adaptada a la curvatura detectada** (más rápido en recta,
más lento en curva cerrada), usa el brazo con cámara para espiar a los
costados y recuperarse si pierde la línea en vez de depender de un barrido
ciego de chasis, y se frena con el lidar frontal si aparece un obstáculo —
sin depender del nodo Python existente.

## Arquitectura

```
/ascamera/.../rgb0/image ──▶ line_follower_node ──┬──▶ /controller/cmd_vel      (velocidad)
 (sensor_msgs/Image)              │                ├──▶ servo_controller        (brazo, SEARCHING)
/ros_robot_controller/imu_raw ───▶│                ├──▶ ~/get_lap_stats         (tuning)
 (giro durante el retorno)        │                └──▶ ~/events internos (RCLCPP_INFO)
/scan_raw ────────────────────────▶│
 (BEST_EFFORT, corte por lidar)    │
/odom_raw ────────────────────────▶│
 (dead-reckoning, lap tracker)     │
                                   │
                    ┌──────────────┴──────────────┐
                    │                              │
             LineTracker                      ZoneMonitor
       (clase pura, sin rclcpp,           (clase pura, sin rclcpp,
        testeada con gtest)               reusada tal cual de B1,
       3 bandas ROI horizontales          testeada con gtest)
       (cerca/media/lejos) → offset_px    sector angular + mediana
       + curvature_rad (0 = recta,        de N scans + normaliza el
       mayor magnitud = curva cerrada)    ángulo 0-360°→-180/180
                    │
                  Pid
          (clase pura, sin rclcpp,
           testeada con gtest)
           PID sobre offset_px → angular.z
```

### Máquina de estados de comportamiento

`IDLE` → `FOLLOWING` ⇄ `SEARCHING` → `STOPPED`, con `STOPPED_OBSTACLE`
interrumpiendo desde cualquier estado:

- **`IDLE`**: motor en cero. Estado inicial tras activar el nodo — activar
  el lifecycle **no** hace que el robot arranque solo; hace falta
  `~/set_running(true)` explícito, y éste se rechaza si el color no está
  calibrado todavía.
- **`FOLLOWING`**: velocidad angular por PID sobre `offset_px`; velocidad
  lineal por `v = clamp(v_max - gain·|curvatura|, v_min, v_max)` cuando las
  3 bandas ven la línea (curvatura confiable), o `0.75·v_max` cuando no
  (no se puede distinguir "recta real" de "no hay suficiente dato" solo con
  `curvatura_rad`, que vale 0 en ambos casos).
- **`SEARCHING`**: se entra si se pierde la línea más de un grace period
  corto. En vez de girar el chasis a ciegas, usa el brazo (cámara montada
  ahí) para espiar a los costados sin mover el robot:
  1. Chasis detenido, brazo barre hacia la derecha en pasos, chequeando
     cada frame — para apenas encuentra la línea (no espera llegar al
     límite).
  2. Si no, barre hacia la izquierda igual.
  3. Si la encontró en algún lado: el chasis gira hacia ese lado **la
     cantidad justa** (integrando yaw real de la IMU hasta que coincide con
     el bearing donde se encontró, no un giro a tiempo fijo) mientras el
     brazo vuelve al centro, y retoma `FOLLOWING`.
  4. Si no la encontró a ningún lado: hoy pasa directo a `STOPPED` (el
     barrido ciego de chasis como último recurso quedó como diseño
     pendiente, no implementado).
- **`STOPPED`**: se recupera con `~/reset` (separado de `~/set_running`,
  que queda con un significado más puro de pausa/arranque normal).
- **`STOPPED_OBSTACLE`**: el lidar frontal anula cualquier comando de
  velocidad sin importar el estado previo. Transitorio: al despejarse el
  obstáculo, vuelve solo (sin `~/set_running` de nuevo) al estado que tenía
  antes de frenar.

### Pose del brazo durante SEARCHING

El brazo se mueve a una pose **fija y pre-calibrada** (`line_follow_arm_pulses`,
encontrada a mano con `tools/arm_teleop.py` del portfolio) cada vez que
arranca `~/set_running(true)`, en vez de calcular una pose por cinemática
inversa en el momento — un diseño más simple terminó siendo más robusto que
intentar capturar "la pose actual" como referencia, que se desalineaba cada
vez que `SEARCHING` se disparaba de nuevo antes de asentarse del todo.

### Lap tracker

Feature extra (no estaba en el objetivo original): mide tiempo, distancia
real recorrida y velocidad promedio por vuelta, usando `/odom_raw`
(dead-reckoning — no es localización real, pero alcanza para comparar
tuning entre pruebas). Arranca a trackear en el primer mensaje de odometría
después de `~/set_running(true)`, y declara "vuelta completa" cuando el
robot se aleja más de `lap_loop_radius_m` del punto de partida y después
vuelve a menos de esa misma distancia. Resultado de la última vuelta
completada disponible en `~/get_lap_stats`.

## Topics/servicios reales que consume/publica

| Dirección | Nombre | Tipo | Notas |
|---|---|---|---|
| Sub | `image_topic` (`/ascamera/camera_publisher/rgb0/image`) | `sensor_msgs/Image` | Único publisher de imagen real en este robot (Aurora930) — no el `/depth_cam/rgb/image_raw` que usa el stock. |
| Sub | `imu_topic` (`/ros_robot_controller/imu_raw`) | `sensor_msgs/Imu` | Único sensor de rotación real e independiente del propio `cmd_vel` comandado. |
| Sub | `scan_topic` (`/scan_raw`) | `sensor_msgs/LaserScan`, QoS `BEST_EFFORT` | Crudo, igual criterio que `lidar_app`/`line_following` del stock. |
| Sub | `odom_topic` (`/odom_raw`) | `nav_msgs/Odometry` | Dead-reckoning puro, solo para el lap tracker. |
| Pub | `cmd_vel_topic` (`/controller/cmd_vel`) | `geometry_msgs/Twist` | |
| Pub | `servo_controller_topic` (`servo_controller`) | `servo_controller_msgs/ServosPosition` | Mueve solo `joint1` durante `SEARCHING`/la pose fija de `FOLLOWING`. |
| Srv | `~/init_finish` | `std_srvs/Trigger` | Convención del resto del stack. |
| Srv | `~/set_running` | `std_srvs/SetBool` | `true` requiere color calibrado; mueve el brazo a la pose fija configurada en cada arranque. |
| Srv | `~/set_target_color` | `interfaces/SetPoint` | Solo aceptado en `IDLE`. |
| Srv | `~/get_target_color` | `std_srvs/Trigger` | |
| Srv | `~/set_threshold` | `interfaces/SetFloat64` | |
| Srv | `~/reset` | `std_srvs/Trigger` | Recupera de `STOPPED`. |
| Srv | `~/get_lap_stats` | `std_srvs/Trigger` | Tiempo/distancia/velocidad promedio de la última vuelta detectada. |

## Parámetros

| Parámetro | Default | Descripción |
|---|---|---|
| `image_topic` | `/ascamera/camera_publisher/rgb0/image` | |
| `cmd_vel_topic` | `/controller/cmd_vel` | |
| `linear_speed_max` / `linear_speed_min` | `0.15` / `0.10` | Rango de velocidad lineal adaptativa. Confirmado en el robot hasta `0.50`/`0.20` (ver CLAUDE.md del repo para el historial de tuning). |
| `curvature_speed_gain` | `0.0` | `v = clamp(v_max - gain·|curvatura_rad|, v_min, v_max)`. `0.0` = velocidad fija en `v_max`. |
| `angular_kp` / `angular_ki` / `angular_kd` | `0.005` / `0.0` / `0.001` | PID angular sobre `offset_px`. Confirmado en el robot con `0.010`/`0.0`/`0.002` hasta 0.50 m/s. |
| `angular_output_max` | `1.0` | Clamp simétrico de la salida del PID. |
| `angular_integral_limit` | `50.0` | Clamp del término integral. |
| `color_sample_patch_px` | `20` | Lado del parche cuadrado muestreado al calibrar color. |
| `lab_l_tolerance` / `lab_ab_tolerance` | `40.0` / `20.0` | Tolerancia del umbral de color en espacio Lab. |
| `min_contour_area` | `30.0` | Área mínima de contorno para considerar detección válida. |
| `near/mid/far_roi_y_start` / `_end` | `0.85-0.95` / `0.55-0.65` / `0.25-0.35` | Bandas ROI horizontales (fracción de alto de imagen). Confirmado en el robot: con el brazo en la pose de `line_follow_arm_pulses`, las que realmente ven la cinta son `0.65-0.75`/`0.55-0.65`/`0.25-0.35`. |
| `search_grace_period_s` | `0.5` | Cuánto tiempo sin ver la línea antes de entrar en `SEARCHING`. |
| `search_timeout_s` | `25.0` | Tiempo total de `SEARCHING` antes de pasar a `STOPPED`. |
| `search_settle_time_s` | `0.3` | Espera tras cada paso del brazo antes de chequear el frame. |
| `search_return_max_s` | `4.0` | Tope de seguridad del giro de retorno (el criterio real es el yaw integrado por IMU, esto es solo un límite por si algo falla). |
| `imu_topic` | `/ros_robot_controller/imu_raw` | |
| `search_arm_max_bearing_deg` | `120.0` | Límite real de la junta base del brazo (joint1, ±120.2° físico). |
| `search_arm_step_deg` | `10.0` | Paso de cada incremento del barrido del brazo. |
| `search_arm_joint1_pulse_sign` | `1.0` | Signo real del pulso de joint1 → bearing. **Confirmar empíricamente en cada robot/montaje** (en el LanderPi de este proyecto es `-1.0`). |
| `search_chassis_turn_speed` | `0.3` | Velocidad angular del giro de retorno del chasis. |
| `servo_controller_topic` | `servo_controller` | |
| `line_follow_arm_pulses` | `[500, 500, 500, 500]` | Pose fija del brazo (joints 1-4) para que la cámara vea bien la línea — encontrar una vez con `tools/arm_teleop.py` del portfolio. |
| `scan_topic` | `/scan_raw` | |
| `obstacle_min_angle_deg` / `obstacle_max_angle_deg` | `-20.0` / `20.0` | Sector angular frontal vigilado por el corte de seguridad. |
| `obstacle_stop_distance_m` | `0.20` | Distancia bajo la cual se considera obstáculo. |
| `obstacle_noise_filter_window` | `3` | Ventana de mediana para filtrar outliers de un solo frame. |
| `odom_topic` | `/odom_raw` | |
| `lap_loop_radius_m` | `0.3` | Radio de detección de "vuelta completa" del lap tracker (un solo radio: salida y regreso). |

## Flujo de configuración de color

La calibración pasa **antes** de `~/set_running(true)`, con el nodo en
`IDLE` (se rechaza si no hay color calibrado). El nodo no abre ninguna
ventana propia — se calibra con un script aparte:

```bash
ros2 run landerpi_adaptive_line_follower line_color_calibration.py
```

Muestra el feed de la cámara; un click sobre la línea llama a
`~/set_target_color` con la posición clickeada. La calibración no persiste
entre reinicios del nodo (fiel al stock).

## Cómo correrlo

```bash
colcon build --packages-select landerpi_adaptive_line_follower
source install/setup.bash
ros2 launch landerpi_adaptive_line_follower line_follower.launch.py
```

## Tests

`LineTracker`, `Pid` y `ZoneMonitor` son clases puras (sin `rclcpp`),
testeadas con `gtest`:

```bash
colcon test --packages-select landerpi_adaptive_line_follower
colcon test-result --verbose
```

## Estado

Las 6 etapas de desarrollo (clase de visión pura → nodo mínimo a velocidad
fija → velocidad adaptativa por curvatura → `SEARCHING` con brazo → corte
de seguridad por lidar → demo completa en circuito con rectas y curvas de
radio distinto) están **confirmadas sobre el robot físico**.

Pendiente / fuera de alcance de esta versión:

- El barrido ciego de chasis como último recurso de `SEARCHING` (si el
  brazo no encuentra la línea a ningún lado) no está implementado — hoy
  pasa directo a `STOPPED`.
