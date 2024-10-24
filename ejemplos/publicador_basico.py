#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# Inicialización del nodo publicador
rospy.init_node('publicador_basico', anonymous=True)

# Creación de un publicador que publica Strings en el topic 'primer_topic'
pub = rospy.Publisher('primer_topic', String, queue_size=10)

# Bucle principal
rate = rospy.Rate(10) # 10hz (10 veces por segundo)
while not rospy.is_shutdown(): # Mientras el maestro no se detenga
    mensaje_publicar = String() # Crear un mensaje de tipo String
    mensaje_publicar.data = f"Hola Mundo {rospy.get_time()}" # Asignar el mensaje a publicar
    pub.publish(hello_str) # Publicar el mensaje
    rate.sleep() # Esperar lo justo para mantener la frecuencia de publicación
