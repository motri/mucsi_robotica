#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray, Int16
from collections import Counter
from itertools import zip_longest

class Robot:
    def __init__(self):
        rospy.init_node('robot_node', anonymous=True)
        rospy.Subscriber('/cartas_detectadas', Int16MultiArray, self.vision_callback)
        self.detected_cards = []  # Última lectura de cartas detectadas
        self.readings = []  # Lecturas acumuladas para varias iteraciones
        rospy.Subscriber('/evalua_tablero', Int16, self.evalua_tablero_callback)
        self.evalua_tablero = 0
        self.pub_tablero_actual = rospy.Publisher('/tablero_actual', Int16MultiArray, queue_size=10)


    def vision_callback(self, msg):
        # Guardar las cartas detectadas en el callback
        self.detected_cards = msg.data
    
    def evalua_tablero_callback(self, msg):
        self.evalua_tablero = msg.data

    def esperar_deteccion(self):
        # Esperar hasta que se detecten cartas
        rospy.loginfo("Esperando detección de cartas...")
        while not rospy.is_shutdown() and not self.detected_cards:
            rospy.sleep(0.1)
        rospy.loginfo(f"Cartas detectadas: {self.detected_cards}")
        return self.detected_cards

    def realizar_varias_lecturas(self, num_lecturas=40):
        # Realizar varias lecturas y acumular los resultados
        self.readings = []
        rospy.loginfo(f"Realizando {num_lecturas} lecturas para mayor precisión...")
        for i in range(num_lecturas):
            self.detected_cards = []
            self.esperar_deteccion()  # Esperar cada lectura
            if self.detected_cards:  # Solo acumular si hay lecturas válidas
                self.readings.append(self.detected_cards)
            rospy.sleep(0.1)  # Pequeña pausa entre lecturas

        rospy.loginfo(f"Lecturas acumuladas: {self.readings}")
        return self.readings

    def calcular_valores_mas_altos(self):
        # Calcular el número más alto para cada posición
        if not self.readings:
            return []

        # Transponer las lecturas acumuladas usando zip_longest
        transposed = list(zip_longest(*self.readings, fillvalue=None))
        
        highest_values = []
        for pos, numbers in enumerate(transposed):
            # Filtrar valores `None` que podrían haber sido añadidos por zip_longest
            filtered_numbers = [num for num in numbers if num is not None]
            highest = max(filtered_numbers)  # Obtener el valor más alto
            highest_values.append(highest)
            rospy.loginfo(f"Posición {pos + 1}, valores: {filtered_numbers}, más alto: {highest}")

        return highest_values

    def revision_cartas(self, cartas_detectadas, cartas_defecto = [6,5,8,7]):
        tecla_pulsada = input(f"¿Han sido detectadas correctamente las cartas (y/n)? Cartas detectadas: {cartas_detectadas}")
        if tecla_pulsada == "y":
            cartas_detectadas
        else:
            cartas_detectadas = cartas_defecto
        print(f"Cartas detectadas: {cartas_detectadas}")
        return cartas_detectadas

    def run(self):
        # Ciclo principal
        while not rospy.is_shutdown():
            if self.evalua_tablero == 1 or self.evalua_tablero == 2:
                print("Empieza")
                rospy.loginfo("Inicio del ciclo")

                self.readings = self.realizar_varias_lecturas(num_lecturas=40)
                cartas_detectadas = self.calcular_valores_mas_altos()
                rospy.loginfo(f"Cartas finales detectadas: {cartas_detectadas}")

                cartas_detectadas =  self.revision_cartas(cartas_detectadas)

                # Publicar los números detectados
                msg = Int16MultiArray()
                msg.data = cartas_detectadas
                self.pub_tablero_actual.publish(msg)

                self.evalua_tablero = 0

if __name__ == '__main__':
    try:
        robot = Robot()
        robot.run()
    except rospy.ROSInterruptException:
        pass