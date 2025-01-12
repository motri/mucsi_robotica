#!/usr/bin/python3

import cv2
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import detector_numero as dn
import os
import subprocess
from copy import deepcopy

bridge = CvBridge()

class NodoCamara:
    def __init__(self):

        #Lanzar el nodo usb_cam automáticamente
        try:
            self.usb_cam_process = subprocess.Popen(["rosrun", "usb_cam", "usb_cam_node"])
            rospy.loginfo("Nodo usb_cam lanzado exitosamente.")
        except Exception as e:
            rospy.logerr(f"Error al lanzar usb_cam_node: {e}")
        
        rospy.init_node('nodo_camara', anonymous=True)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.__cb_image)
        self.pub = rospy.Publisher('/cartas_detectadas', Int16MultiArray, queue_size=10)
        self.cv_image = None
            
        # Obtener la ruta al archivo de calibración
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.calibration_data_path = os.path.join(base_dir, "static/calibration_data.npz")

        # Cargar los valores de calibración de la cámara
        with np.load(self.calibration_data_path) as data:
            self.loaded_mtx = data['camera_matrix']
            self.loaded_dist = data['dist_coeffs']

        self.calibracion = False  # Variable para activar/desactivar calibración

    def __cb_image(self, image: Image):
        self.cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

    def run(self):
        cv2.namedWindow('VentanaThresh')
        cv2.namedWindow('VentanaFrame')

        while not rospy.is_shutdown():
            if self.cv_image is None:
                rospy.sleep(0.1)
                continue

            # Copiar la imagen para evitar modificaciones directas
            frame = deepcopy(self.cv_image)

            # Si la calibración está activada, corregir la distorsión de la imagen
            if self.calibracion:
                frame = cv2.undistort(frame, self.loaded_mtx, self.loaded_dist, None, self.loaded_mtx)

            # Convertir la imagen a escala de grises
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Reducir ruido y umbralizar la imagen
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV)

            # Encontrar contornos
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Dibujar los contornos en la imagen para depuración
            cv2.imshow("VentanaThresh", thresh)

            # Ordenar los contornos por área
            if contours:
                contours = sorted(contours, key=cv2.contourArea, reverse=True)

            # Eliminar el contorno más grande (probablemente el borde de la ventana)
            if contours:
                contours.pop(0)

            # Filtrar los contornos relevantes (posibles cartas)
            if len(contours) > 1:
                media = np.mean([cv2.contourArea(c) for c in contours])
                desviacion = np.std([cv2.contourArea(c) for c in contours])
                contours = [c for c in contours if cv2.contourArea(c) > media + 2 * desviacion]

            # Ordenar las cartas de izquierda a derecha
            card_positions = [(cv2.boundingRect(c)[0], c) for c in contours]
            card_positions.sort(key=lambda x: x[0])  # Ordenar por coordenada X (posición horizontal)

            # Detectar el número de cada carta
            card_numbers = []
            for _, contour in card_positions:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int32(box)

                # Dibujar contornos
                cv2.drawContours(thresh, [box], 0, (0, 0, 255), 3)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)

                # Detectar el número usando la función de detector_numero
                number = dn.process_card_box(box, frame, thresh, figura=False)

                if isinstance(number, int):  # Verifica que sea un entero
                    card_numbers.append(number)


            # Publicar los números detectados
            msg = Int16MultiArray()
            #msg.data = card_numbers
            #self.pub.publish(msg)
            msg.data = [int(num) for num in card_numbers if isinstance(num, int)]  # Filtrar valores no enteros
            self.pub.publish(msg)


            # Mostrar la imagen procesada
            cv2.imshow("VentanaFrame", frame)

            # Salir si se presiona 'q'
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        nodo = NodoCamara()
        nodo.run()
    except rospy.ROSInterruptException:
        pass
