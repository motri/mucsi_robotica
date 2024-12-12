import cv2
import numpy as np

# Función para procesar la región de cada carta
def process_card_box(box, frame, thresh, figura):

    # Obtener las coordenadas del rectángulo delimitador de la caja
    x, y, w, h = cv2.boundingRect(box)

    # Extraer la región de interés (ROI) de la imagen umbralizada
    box_region = thresh[y:y + h, x:x + w]

    # Verificar que el tamaño del contorno es válido
    if box_region.size > 0:  

        # Verificar el número de canales de la imagen
        if len(box_region.shape) == 3 and box_region.shape[2] == 3:
            # Convertir la imagen a escala de grises si tiene 3 canales (BGR)
            box_gray = cv2.cvtColor(box_region, cv2.COLOR_BGR2GRAY)
        else:
            # Si la imagen ya está en escala de grises, no es necesario convertirla
            box_gray = box_region

        # Calcular la intensidad media de la imagen
        mean_intensity = np.mean(box_gray)

        # Calcular los umbrales para el detector de bordes de Canny de manera dinámica
        low_threshold = max(30, int(mean_intensity * 0.5))
        high_threshold = min(150, int(mean_intensity * 1.5))

        # Aplicar el detector de bordes de Canny
        canny_edges = cv2.Canny(box_gray, low_threshold, high_threshold)

        # Aplicar una operación de cerrado morfológico para asegurar que los bordes estén bien cerrados
        kernel = np.ones((5, 5), np.uint8)
        closed = cv2.morphologyEx(canny_edges, cv2.MORPH_CLOSE, kernel)

        # Encontrar los contornos en la imagen cerrada
        inner_contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calcular las dimensiones del rectángulo central
        height, width = box_region.shape[:2]
        rect_width = int(width * 0.95)
        rect_height = int(height * 0.95)

        # Definir las coordenadas del rectángulo central
        top_left = (width // 2 - rect_width // 2, height // 2 - rect_height // 2)
        bottom_right = (width // 2 + rect_width // 2, height // 2 + rect_height // 2)

        # Dibujar el rectángulo central en el marco principal
        cv2.rectangle(frame, (x + top_left[0], y + top_left[1]), (x + bottom_right[0], y + bottom_right[1]), (0, 255, 0), 2)

        # Filtrar los contornos que están completamente dentro del rectángulo central
        contours_in_rectangle = []
        for contour in inner_contours:
            inside_rectangle = True
            for point in contour:
                if not (top_left[0] <= point[0][0] <= bottom_right[0] and top_left[1] <= point[0][1] <= bottom_right[1]):
                    inside_rectangle = False
                    break
            if inside_rectangle:
                contours_in_rectangle.append(contour)

        # Calcular las áreas de los contornos filtrados y ordenarlos de mayor a menor
        areas = [(cv2.contourArea(contour), contour) for contour in contours_in_rectangle]
        areas = sorted(areas, key=lambda x: x[0], reverse=True)
        significant_contours = []
        if len(areas) > 0:
            prev_area = areas[0][0]
            significant_contours.append(areas[0])  # Agregar siempre el contorno más grande
            for i in range(1, len(areas)):
                current_area = areas[i][0]
                if current_area == 0:  # Evitar división por cero
                    continue
                # Detectar un salto significativo en el área
                if prev_area / current_area > 2:
                    break
                significant_contours.append(areas[i])
                prev_area = current_area

        # Obtener los contornos significativos
        filtered_contours = [contour for _, contour in significant_contours]
        count = len(filtered_contours)

        # Determinar si la carta es una figura o un número según el número de contornos significativos
        if figura:  # Umbral ajustable para distinguir entre figuras y números
            text = 'Figura'
        else:
            # Dibujar los contornos filtrados en el marco principal
            for contour in filtered_contours:
                contour[:, 0, 0] += x  # Desplazar en X
                contour[:, 0, 1] += y  # Desplazar en Y
                cv2.drawContours(frame, [contour], -1, (0, 0, 255), 3)
            text = f'Numero: {count}'
        
        # Configuración de texto para mostrar el resultado
        font_scale = 1
        font_thickness = 2
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0]
        text_width, text_height = text_size
        text_x = x + top_left[0] + (rect_width // 2) - (text_width // 2)
        text_y = y + top_left[1] - 10
        # Dibujar el texto en el marco principal
        cv2.putText(frame, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 0, 0), font_thickness, cv2.LINE_AA)

        return count