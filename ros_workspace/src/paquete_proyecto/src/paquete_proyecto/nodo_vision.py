#!/usr/bin/python3

import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray
import detector_numero as dn
import os

def main():

    # Inicializar el nodo de ROS
    rospy.init_node('vision_node', anonymous=True)
    pub = rospy.Publisher('/cartas_detectadas', Int32MultiArray, queue_size=10)

    # Obtener la ruta al archivo de calibración
    base_dir = os.path.dirname(os.path.abspath(__file__))
    calibration_data_path = os.path.join(base_dir, "static/calibration_data.npz")

    # Cargar los valores de calibración de la cámara
    with np.load(calibration_data_path) as data:
        loaded_mtx = data['camera_matrix']
        loaded_dist = data['dist_coeffs']

    # Inicializar la cámara
    cap = cv2.VideoCapture(0)

    # Variables para la calibración (activar/desactivar)
    calibracion = False

    # Leer un frame de la cámara
    success, frame = cap.read()
    if not success:
        rospy.logwarn("No se pudo leer el frame de la cámara")
        return

    # Si la calibración está activada, corregir la distorsión de la imagen
    if calibracion:
        frame = cv2.undistort(frame, loaded_mtx, loaded_dist, None, loaded_mtx)

    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Reducir ruido y umbralizar la imagen
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV)

    # Encontrar contornos
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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

        # Detectar el número usando la función de detector_numero
        number = dn.process_card_box(box, frame, thresh, figura=False)
        card_numbers.append(number)

    # Publicar los números detectados
    msg = Int32MultiArray()
    msg.data = card_numbers
    pub.publish(msg)

    # Liberar la cámara
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
