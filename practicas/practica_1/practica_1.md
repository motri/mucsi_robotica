# Práctica ROS: Control de Robot con MoveIt y primeros pasos en ROS

## Entorno de Trabajo

Se utilizará el entorno de trabajo dockerizado proporcionado en este repositorio. En concreto, esta práctica se realizará en un contenedor de desarrollo que parta de la imagen `desktop`.

Si se cree conveniente, es posible modificar la imagen de Docker para añadir dependencias adicionales que se vayan a utilizar en el futuro proyecto. Esto se valorará positivamente.

## Descripción de la práctica

Esta práctica se centrará en el desarrollo de un nuevo paquete de ROS en el espacio de trabajo de ROS proporcionado en el entorno de desarrollo. Este paquete deberá contener como mínimo 2 nodos de ROS:

1. Nodo control robot:

Este nodo deberá ser capaz de, a partir de la clase de control desarrollada durante la asignatura:

- Añadir obstáculos a la escena de planificación (idealmente el suelo).

- Mover el robot a una pose (posición y orientación del extremo del robot).

- Mover el robot a una configuración (ángulos de los actuadores del robot).

- Mover el extremo del robot por una trayectoria dada.

Este nodo deberá estar suscrito al topic `/consignas` donde recibirá las órdenes que debe realizar, y en función de esta orden realizara alguna de las tareas anteriores.

2. Nodo de órdenes:

Este nodo deberá publicar órdenes de manera interactiva en el topic `/consignas` para que el nodo de control del robot las ejecute conforme a los 4 requisitos anteriores. Queda a criterio del desarrollador mapear estas opciones a los mensajes que serán enviados. Estas órdenes serán de tipo `std_msgs/Int`.

La acción a realizar en cada momento se pedirá por la terminal al usuario.

Para probar la funcionalidad de los nodos, será posible lanzar los scripts de Python en modo depuración, sin necesidad de generar un launchfile.

## Opcional

Opcionalmente en vez de interactuar con un único topic, trabajar con 4 topics diferentes para cada una de las acciones:

- Topic `/mover_pose`: Intercambiará mensajes de tipo `/geometry_msgs/Pose` para mover el robot a una pose específica.

- Topic `/mover_configuracion`: Intercambiará mensajes de tipo `/sensor_msgs/JointState` para mover el robot a una configuración específica.

- Topic `/trayectoria_cartesiana`: Intercambiará mensajes de tipo `/geometry_msgs/PoseArray` para ejecutar una trayectoria que haga que el extremo del robot pase por todas estas poses.

- Topic `/añadir_obstaculo`: Intercambiará mensajes de un tipo propio que se debe definir. Este tipo será una combinación de un float y una pose. El float definirá el lado del cubo que se creará y la pose, la pose donde se creará.

## Evaluación

La práctica será evaluada de acuerdo a los siguientes criterios:

Creación y estructura del paquete de ROS: Organización del código y correcta configuración del paquete.

Capacidad de mover el robot: Se evaluará la correcta implementación de los movimientos a una pose, configuración y trayectorias cartesianas.

Manipulación de la escena: Se valorará la habilidad de añadir obstáculos y planificar movimientos evitando colisiones.

El apartado opcional será únicamente sumativo, es decir, se puede obtener la máxima nota sin hacerlo. Si se realiza este apartado, la nota obtenida se sumará a la obtenida en la práctica.

## Entrega

La práctica se entregará en un fichero comprimido que deberá contener el repositorio completo junto con las modificaciones realizadas.

Se entiende como repositorio completo tanto el espacio de trabajo de ROS como los ficheros relacionados con los contenedores de Docker.

### **Enlaces de interés:**
En el repositorio existen [scripts de ejemplo básicos](./ejemplos) para desarrollar publicadores y suscriptores en ROS, así como la clase de control de robots desarrollada en clase. 

[Aquí](http://wiki.ros.org/ROS/Tutorials) se pueden encontrar tutoriales detallados sobre como crear paquetes de ROS, mensajes customizados, etc