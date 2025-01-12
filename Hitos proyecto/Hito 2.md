# Segundo Hito: Captura y Visualización de Señales del Robot

Esta práctica se centrará en el desarrollo de un nuevo paquete de ROS en el espacio de trabajo de ROS proporcionado en el entorno de desarrollo. Este paquete deberá contener como mínimo 1 nodo de ROS.

Este nodo deberá ser capaz de:

- Obtener, las variables de velocidad, configuración y esfuerzo de las trayectorias ejecutadas con un sampling de 100 ms al 100% de la velocidad del robot.
- Registrar los datos obtenidos de cada trayectoria como una medida en la instancia de influxDB proporcionada por Gestamp.

Para esto, será necesario que el nodo esté suscrito al topic `/joint_states` donde se publica la información requerida a 500 Hz (2 ms).

1. **Diseño y desarrollo del nodo**: 
   
   Durante la fase de diseño del nodo será necesario establecer cuál va a ser el comportamiento del nodo, es decir, sería conveniente plantear y resolver como mínimo las siguientes cuestiones:
    
    - ¿La captura de datos va a ser síncrona o asíncrona?, es decir:
    
        - Si se plantea como asíncrono, el nodo debería estar constantemente comprobando si el robot se está moviendo y cuando se empiece a mover registrar la trayectoria automáticamente.
        - Si se plantea como síncrono, el nodo debería esperar a una señal externa (que debería provenir de otro nodo a través de un topic) para empezar a capturar los datos y a otra para terminar la captura y registrarlos en la base de datos.
    - ¿Debe interactuar este nodo con algún otro nodo del sistema?, y si es así, ¿Cómo?
    - Para el registro de los datos en la DB, ¿Se va a utilizar un timestamp relativo (desde que empieza la trayectoria) o absoluto (desde epoch)?

Las medidas registradas en influxDB deberán tener asociados, además del timestamp que está asociada de manera nativa en influxDB, los siguientes datos:
```json
{
    "TraceabilityCode": LLLAAYYJJJHHMMSS (Int),
    "VelocityValues": [[Axis01, Axis02, Axis03, Axis04, Axis05, Axis06], ...](Float),
    "VelocityUnits": "rad/s" (String),
    "RotationValues": [[Axis01, Axis02, Axis03, Axis04, Axis05, Axis06], ...](Float),
    "RotationUnits": "rad",
    "TorqueValues": [[Axis01, Axis02, Axis03, Axis04, Axis05, Axis06], ...](Float),
    "TorqueUnits": "N·m" (String)
}
```

2. **Ejecución de trayectorias asociadas a soldadura de solape**:
    
    Una vez desarrollado el nodo, planificar y ejecutar 6 trayectorias (3 correctas y 3 con colisión/fallo durante la ejecución) asociadas a soldadura de solape. Para esto, planificar una trayectoria en la que el efector final del robot se mueva de manera lineal a través de varias poses predefinidas.

    ![Dibujo de trayectorias de soldadura](/pictures/dibujo_trajs_soldadura.png)

    Comprobar el correcto funcionamiento del nodo registrando los datos de las 6 trayectorias en la base de datos y visualizarlas en grafana.

3. **Documentación de la aplicación**:
   
   Elaborar una breve documentación que describa el funcionamiento del nodo. Esto incluirá:
     - Los nodos con los que interactúa.     
     - Los topics a los que está suscrito y sobre los que publica (si es que fuera necesario)
     - Cómo se integra con la estructura inicial planteada para el proyecto.
     - Cómo se han generado, planificado y ejecutado las 6 trayectorias.
     - Visualización de los datos recogidos para las 6 trayectorias ejecutadas.

## Entrega
La entrega del segundo hito se integrará en el repositorio de git utilizado para el hito 1 y debe incluir el workspace estructurado, las imágenes de Docker preparadas y la documentación correspondiente. 

La entrega se realizará en el curso ALUD.
## Evaluación
Se evaluará:
- La correcta organización e integración del nodo en el entorno de trabajo.
- La claridad de la documentación y la calidad tanto del nodo como de las trayectorias desarrolladas.
- El correcto formato y visualización de los datos.

## Opcional
Opcionalmente, se podrán programar estadísticas y alertas en Grafana que muestren diferencias de comportamiento en las trayectorias simuladas.
