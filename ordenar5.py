# Imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from actionlib import SimpleActionClient
from time import sleep

# Clase para controlar las funciones del robot
class ControlRobot:

    # Constructor de la clase para controlar las funciones del robot
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
        self.add_floor()

    # Devolvemos una lista de los angulos de los motores
    def get_motor_angles(self) -> list:
        return self.move_group.get_current_joint_values()
    
    # Movemos los motores del robot a la posicion que le indicamos
    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    # Obtenemos la pose del efector final del robot
    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    # Mover el efector final del robot a una pose indicada, devuelve True al finalizar
    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def pose_actual(self) -> Pose:
        return self.move_group.get_current_pose().pose
    
    def mover_a_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)
    
    # Añadimos un obstaculo, en este caso una caja
    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    # Añadimos el suelo
    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2,2,.05)) # Editar valores con medidas reales

    # Indicamos una trayectoria al robot, devuelve True al finalizar
    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, True)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait = wait)
    
    
    def moverConfigBase(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-1.8235538641559046, -1.505070039337017, -0.7248993515968323, -2.4801069698729457, 1.5716415643692017, 0.05282888934016228]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfig1(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [0.5408084988594055, -1.969069620172018, -0.690409779548645, -2.046171327630514, 1.5728977918624878, 1.482495903968811]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfig2(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [0.15196920931339264, -2.0932113132872523, -0.46324196457862854, -2.142572065392965, 1.5742270946502686, 1.0903068780899048]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)

    def moverConfig3(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.22814971605409795, -2.1172615490355433, -0.4825994670391083, -2.1109215221800746, 1.5729188919067383, 0.7840540409088135]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfig4(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.6141679922686976, -1.9406167469420375, -0.5403124094009399, -2.230209013024801, 1.5725231170654297, 0.31881219148635864]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)

    def moverConfigComodin(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [0.06632380932569504, -1.4258058828166504, -1.3058303594589233, -1.9801088772215785, 1.573224425315857, 0.988903284072876]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()        
        return result.reached_goal
    
    def abrir_pinza(self):
        control = ControlRobot()
        print("Abriendo la pinza...")
        control.mover_pinza(100.0, 5.0)  # Abrir a 10cm con 5N de fuerza
        sleep(2)


    def cerrar_pinza(self):
        control = ControlRobot()
        print("Cerrando la pinza...")
        control.mover_pinza(60.0, 5.0)  # Cerrar a 6cm con 5N de fuerza
        sleep(4)

    
    def bajar_pinza(self):
        pose_actual = control.pose_actual()
        pose_actual.position.z -= 0.07 # Ajustar a la medida real
        control.move_trajectory([pose_actual])
        sleep(2)

    
    def subir_pinza(self):
        print("subiendo pinza")
        pose_actual = control.pose_actual()
        pose_actual.position.z += 0.07   # Ajustar a la medida real
        control.move_trajectory([pose_actual])
        sleep(2)
    

if __name__ == '__main__':
    try:
        # Inicializar el control del robot
        control = ControlRobot()

        control.add_floor() # Quedaría por medir como es el suelo en la escena real 

        #control.moverConfigBase()
        #control.abrir_pinza()

        control.moverConfigBase()
        control.subir_pinza()
        
        
        #Ordenar con robot
        #--------------------------------------------------------
        # funcion que tomando el array ordwenado y sin ordenar nos devuelva pose_ordenado y pose_sin_ordenar
        # se puede hacer con un diccionario
        """
        pose_sin_ordenar = [pose2,pose1,pose3,pose4,posevacia]
        pose_ordenado = [pose4,pose3,pose2,pose1,posevacia]
        
        posiciones_bin= [1,1,1,1,0]

        dictposes_binario={

        }


        dictposes_numeros={pose_ordenado[0]: array_sin_ordenar[0] ,
               pose_ordenado[1]:array_sin_ordenar[1],
               pose_ordenado[2]:array_sin_ordenar[2]
               pose_ordenado[3]:array_sin_ordenar[3]}


        array_sin_ordenar=['2','1','3','4']
        array_sin_ordenar.append('0')#0 es el comodin ,para añadir el comodin a el array a el final
        array_sin_ordenar=['2','1','3','4','0']

        array_ordenado=['4','3','2','1']
        array_ordenado.append('0')
        array_ordenado=['4','3','2','1','0']

        for i in array_ordenado:
            if array_ordenado[i]!=array_sin_ordenar[i]:
                for j in posiciones_bin:
                    if posiciones_bin[j]==0:
                        a = array_ordenado[i]
                        b = posiciones_bin[j]
                        c = array_sin_ordenar[i]
                        # bubble_sort(a, b, c, pose_ordenado[i], pose_ordenado[4], pose_sin_ordenar[i])




        """
        #-----------------------------------------------------

        # Tenemos cartas en config 1 y 2 y usamos config3 como auxiliar
        # Vamos a mover asi:
        # Carta en config 1 --> config 3
        # Carta en config 2 --> config 1
        # Carta en config 3 --> config 2

        control.moverConfig1()
        print(control.get_motor_angles())
        control.abrir_pinza()
        control.bajar_pinza()
        control.cerrar_pinza()
        control.subir_pinza()

        #control.moverConfigBase()
        #control.subir_pinza()

        control.moverConfig3()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.abrir_pinza()
        control.subir_pinza()

        #control.moverConfigBase()
        #control.subir_pinza()

        control.moverConfig2()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.cerrar_pinza()
        control.subir_pinza()

        #control.moverConfigBase()
        #control.subir_pinza()

        control.moverConfig1()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.abrir_pinza()
        control.subir_pinza()

        #control.moverConfigBase()
        #control.subir_pinza()

        control.moverConfig3()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.cerrar_pinza()
        control.subir_pinza()

        #control.moverConfigBase()

        control.moverConfig2()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.abrir_pinza()
        control.subir_pinza()

        control.moverConfigBase()
        print(control.get_motor_angles())

        # TODO
        # Registrar los objetos que hay en la escena, la base, los soportes de las cartas 
        # Coger las configuraciones encima de la carta, y bajar en y a por ella, y luego volver a subir
        # Utilizar un archivo yml para leer las configuraciones
        
    except rospy.ROSInterruptException:
        print("Programa interrumpido")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        moveit_commander.roscpp_shutdown()
        print("Programa finalizado")
        