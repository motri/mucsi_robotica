import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose
import time

class NodoOrdenes:
    def __init__(self):
        rospy.init_node('ordenes', anonymous=True)
        self.pub_trayectoria = rospy.Publisher('/trayectoria_cartesiana', PoseArray, queue_size=10)

        # Define trajectories
        self.trayectorias = [
            [
                [0.3, 0.0, 0.3],
                [0.3, 0.0, 0.35],
                [0.325, 0.0, 0.35],
                [0.325, 0.0, 0.3]
            ],
            [
                [0.325, 0.0, 0.3],
                [0.325, 0.0, 0.35],
                [0.35, 0.0, 0.35],
                [0.35, 0.0, 0.3]
            ],
            [
                [0.35, 0.0, 0.3],
                [0.35, 0.0, 0.35],
                [0.375, 0.0, 0.35],
                [0.375, 0.0, 0.3]
            ],
            [
                [0.375, 0.0, 0.3],
                [0.375, 0.0, 0.35],
                [0.4, 0.0, 0.35],
                [0.4, 0.0, 0.3]
            ],
            [
                [0.4, 0.0, 0.3],
                [0.4, 0.0, 0.35],
                [0.425, 0.0, 0.35],
                [0.425, 0.0, 0.3]
            ],
            [
                [0.425, 0.0, 0.3],
                [0.425, 0.0, 0.35],
                [0.45, 0.0, 0.35],
                [0.45, 0.0, 0.3]
            ]
        ]

    def enviar_trayectoria(self, trayectoria):
        """
        Publica una trayectoria específica en el topic /trayectoria_cartesiana.
        """
        pose_array = PoseArray()
        for pose_coords in trayectoria:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pose_coords
            # Assign neutral orientation
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self.pub_trayectoria.publish(pose_array)
        rospy.loginfo("Trayectoria publicada en /trayectoria_cartesiana.")

    def empezar_soldadura(self):
        """
        Itera sobre todas las trayectorias con un intervalo de 2 segundos entre cada una,
        y vuelve a preguntar al usuario después de completar todas las trayectorias.
        """
        while not rospy.is_shutdown():
            print("¿Desea empezar la soldadura? (1: Sí, cualquier otra tecla: No)")
            respuesta = input("Ingrese su opción: ").strip()
            if respuesta == '1':
                for i, trayectoria in enumerate(self.trayectorias):
                    rospy.loginfo(f"Ejecutando trayectoria {i + 1}...")
                    self.enviar_trayectoria(trayectoria)
                    time.sleep(2)  # Intervalo entre trayectorias
                rospy.loginfo("Soldadura completada.")
            else:
                rospy.loginfo("Operación cancelada por el usuario. Vuelve a preguntar.")
                time.sleep(1)  # Small delay before reasking


if __name__ == '__main__':
    try:
        nodo_ordenes = NodoOrdenes()
        nodo_ordenes.empezar_soldadura()
    except rospy.ROSInterruptException:
        pass
