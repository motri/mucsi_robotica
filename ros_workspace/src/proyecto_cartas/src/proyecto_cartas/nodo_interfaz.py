#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

class NodoMenu:
    def __init__(self):
        # Inicialización del nodo publicador
        rospy.init_node('nodo_ordenes', anonymous=True)
        # Creación de un publicador que publica Int16 en el topic 'ordenes'
        self.pub_inicia_juego = rospy.Publisher('/ordenes', Int16, queue_size=10)
        # Suscripción al estado del juego
        rospy.Subscriber('/estado_juego', Int16, self.estado_juego_callback)
        self.estado_juego = 2  # Estado inicial esperando elección del juego
    
    def estado_juego_callback(self, msg):
        # Guardar el estado del juego cuando se recibe un mensaje
        self.estado_juego = msg.data

    def run(self):
        # Bucle principal
        rate = rospy.Rate(10)  # 10hz (10 veces por segundo)
        while not rospy.is_shutdown():  # Mientras el maestro no se detenga
            if self.estado_juego == 2:  # Estado 2: esperando la elección del juego
                print("Escoge el modo de juego que desees (1/2):")
                print("1: Ordenar Cartas\n2: BlackJack") 
                try:
                    orden = int(input())  # Obtener entrada del usuario
                    if orden == 1:
                        print("Juego seleccionado: Ordenar Cartas")
                        self.pub_inicia_juego.publish(orden)
                    elif orden == 2:
                        print("Juego seleccionado: BlackJack")
                        self.pub_inicia_juego.publish(orden)
                    else:
                        print("Juego seleccionado no disponible.")
                except ValueError:
                    print("Por favor ingrese un número válido.")

            elif self.estado_juego == 0:  # Estado 0: error
                print("Ha habido un error, reiniciando...")
                self.estado_juego = 2  # Restablece el estado al estado de espera

            print()  # Espacio en blanco para mayor claridad
            rate.sleep()  # Mantener la tasa de publicación de 10hz

if __name__ == '__main__':
    nodo_menu = NodoMenu()
    nodo_menu.run()
