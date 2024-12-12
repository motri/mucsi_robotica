import rospy
from std_msgs.msg import Int16MultiArray, Int16
from typing import List

class CardSorterNode:
    def __init__(self):
        rospy.init_node("card_sorter", anonymous=True)
        self.pub = rospy.Publisher("/moveCard", Int16MultiArray, queue_size=10)
        rospy.Subscriber("/tablero_actual", Int16MultiArray, self.current_order_callback)
        rospy.Subscriber("/ordenes", Int16, self.iniciar_juego)
        self.pub_evalua_tablero = rospy.Publisher("/evalua_tablero", Int16, queue_size=10)
        self.confirmation_received = False
        self.juego_actual = 0

        # Subscriber to confirmation topic
        rospy.Subscriber("/moveStatus", Int16, self.confirmation_callback)

    def iniciar_juego(self, msg):
        if msg.data == 1:
            self.juego_actual = 1
            self.pub_evalua_tablero.publish(msg)
        

    def confirmation_callback(self, status):
        """
        Callback to listen for confirmation on /moveStatus.
        """
        if status.data == 1:
            rospy.loginfo("Acción confirmada en /moveStatus.")
            self.confirmation_received = True

    def publish_and_wait_for_confirmation(self, message):
        """
        Publica un mensaje en el topic /movecard y espera confirmación en /moveStatus.
        """
        self.confirmation_received = False

        # Publicar el mensaje
        self.pub.publish(message)
        rospy.loginfo(f"Publicado: {message.data}")

        # Esperar hasta recibir la confirmación
        rate = rospy.Rate(10)  # 10 Hz
        while not self.confirmation_received and not rospy.is_shutdown():
            rate.sleep()

        if not self.confirmation_received:
            rospy.logwarn("No se recibió confirmación en /moveStatus antes de timeout.")

    def sort_cards(self, current_order: List[int], desired_order: List[int]):
        """
        Ordena las cartas en el robot de acuerdo a las reglas descritas.
        """
        for current_index in range(len(current_order)):
            if current_order[current_index] == desired_order[current_index]:
                continue  # Pasamos al siguiente índice si ya está en orden

            # Guardar valores actuales y deseados
            current_value = current_order[current_index]
            expected_value = desired_order[current_index]

            # Encontrar el índice del valor esperado en el arreglo actual
            expected_value_index = current_order.index(expected_value)

            # Paso 1: Publicar current_index -> 4
            message = Int16MultiArray()
            message.data = [current_index, 4]
            self.publish_and_wait_for_confirmation(message)

            # Paso 2: Publicar expected_value_index -> current_index
            message.data = [expected_value_index, current_index]
            self.publish_and_wait_for_confirmation(message)

            # Paso 3: Publicar 4 -> expected_value_index
            message.data = [4, expected_value_index]
            self.publish_and_wait_for_confirmation(message)

            # Actualizar el current_order
            current_order[expected_value_index] = current_value

    def current_order_callback(self, msg):
        """
        Callback para recibir el orden actual desde un topic.
        """
        if self.juego_actual == 1:
            current_order = list(msg.data)
            rospy.loginfo(f"Orden actual recibido: {current_order}")
            desired_order = sorted(current_order)
            self.sort_cards(current_order, desired_order)

    def run(self):
        """
        Mantén el nodo en funcionamiento.
        """
        rospy.spin()

if __name__ == "__main__":
    try:
        node = CardSorterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass