# landerpi_adaptive_line_follower

Proyecto de portfolio (B2) sobre el robot **LanderPi** (base mecanum + brazo
de 6 GDL + lidar MS200/LD19): un seguidor de línea en C++ que reemplaza
`line_following.py` del stack Python de fábrica (que sigue la línea a
velocidad fija y se detiene en seco si la pierde). Esta versión estima la
curvatura de la línea para adaptar la velocidad, usa el brazo con cámara
para "espiar" a los costados y recuperarse si pierde la línea en una curva
cerrada, y se frena con el lidar si aparece un obstáculo adelante.

Paquete:

- [`landerpi_adaptive_line_follower`](landerpi_adaptive_line_follower/README.md)
  — el nodo en sí (`ament_cmake`, C++). Ver su README para arquitectura
  completa, parámetros, flujo de calibración de color, y cómo correrlo/
  testear.

## Contexto técnico

Desarrollado y probado en hardware real (LanderPi corriendo ROS 2 Humble
dentro de un contenedor Docker en una Raspberry Pi). Todas las integraciones
(topics, servicios, convenciones de mensajes) fueron verificadas contra el
estado real y en vivo del robot, no solo contra la documentación o el código
fuente del stock — por ejemplo, el topic real de la cámara
(`/ascamera/camera_publisher/rgb0/image`) difiere del que usa
`line_following.py` del stock (`/depth_cam/rgb/image_raw`, que no tiene
ningún publisher en este modelo de cámara).

Las 6 etapas de desarrollo (clase pura de visión → nodo mínimo a velocidad
fija → velocidad adaptativa por curvatura → búsqueda con el brazo → corte de
seguridad por lidar → demo completa) fueron probadas de forma incremental y
acumulativa sobre el robot físico — no en simulación.

## Build

Pensado para vivir en `src/` de un workspace `colcon` junto al resto del
stack del robot (necesita, como dependencias externas del propio stock:
`interfaces`, `servo_controller_msgs`):

```bash
colcon build --packages-select landerpi_adaptive_line_follower
colcon test --packages-select landerpi_adaptive_line_follower
```
