# Primer Hito: Estructura del Sistema Distribuido en ROS para el Control de un Brazo Robótico UR3e

El objetivo de este hito es garantizar que se cuenta con un entorno de trabajo bien estructurado y documentado, donde se puedan implementar algoritmos avanzados sobre el brazo robótico UR3e. Este hito consiste en las siguientes actividades:

1. **Estructura del Workspace de ROS**: 
   - Crear uno o varios paquetes de ROS vacíos para ir integrando las funcionalidades en un futuro.

2. **Preparación de las imágenes de Docker**:
   - Configurar y construir las imágenes de Docker necesarias para el proyecto. Estas imágenes deben permitir el trabajo de manera local, replicando el entorno de laboratorio del aula de robótica.
   - Incluir todas las dependencias necesarias para trabajar con ROS y las librerías del UR3e. Se espera que los estudiantes logren generar imágenes que puedan desplegarse fácilmente tanto en los PCs del laboratorio como en sus entornos locales.

3. **Documentación de la Estructura del Sistema**:
   - Elaborar una breve documentación que describa la estructura del sistema distribuido. Esto incluirá:
     - **Nodos ROS**: Definir qué nodos formarán parte del sistema. Cada nodo debe tener una función clara, como la percepción (procesamiento de sensores), el control del movimiento del brazo, o la planificación de trayectorias.
     - **Comunicación entre Nodos**: Describir cómo se van a comunicar los nodos, especificando si la comunicación será sincróna o asincróna. Es necesario identificar los canales de comunicación ROS, tales como `topics`, `services` o `actions`, así como los tipos de mensajes utilizados.
     - **Tipos de Datos**: Definir los tipos de datos que serán intercambiados entre nodos. Se espera que los estudiantes definan los mensajes relevantes, incluyendo los campos necesarios para la correcta transmisión de información (por ejemplo, posiciones, velocidades, señales de sensores, etc.).

## Entrega
La entrega del primer hito se integrará en un repositorio de git hosteado en GitHub y debe incluir el workspace estructurado, las imágenes de Docker preparadas, y la documentación correspondiente. 

La entrega se realizará en el curso ALUD.
## Entrega
Se evaluará la correcta organización del entorno de trabajo, la claridad de la documentación y la capacidad de los estudiantes para configurar un sistema distribuido en ROS de manera efectiva.
