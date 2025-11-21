import cv2
import numpy as np
import requests
import time
from urllib.request import urlopen
from ultralytics import YOLO
import csv
import os
from datetime import datetime
from typing import Optional, Tuple
import math

"""
Sistema de Detección y Navegación Autónoma
_________________________________________________________________________

MÓDULO: FINAL.py

AUTORES:
  - MENDOZA MENDOZA IVAN            (Líder del proyecto, procesamiento de imagenes y documentación)
  - MARTÍNEZ SÁNCHEZ JOSELYN        (Control de motores y ESP32)
  - MORENO CHÁVEZ IVONNE AZENETH    (Visión computacional)
  

INSTITUCIÓN: Benemérita Universidad Autónoma de Puebla
FACULTAD: Ciencias de la Electróncia
CARRERA: Licenciatura en Ingeniería Mecatrónica
PROYECTO: Agente Hardware Fase 1

FECHA DE CREACIÓN: 10 de octubre de 2025
ÚLTIMA MODIFICACIÓN: 12 de noviembre de 2025
VERSIÓN: 1.0.0

DESCRIPCIÓN:
Este módulo se encarga del procesamiento de video en tiempo real, incluyendo
detección de personas, cálculo de distancias, y control del sistema de navegación
del robot autónomo.

LICENCIA: MIT License
Copyright (c) 2025 IVAN MENDOZA, JOSELYN MARTÍNEZ, IVONNE AZENETH MORENO

Uso permitido para fines académicos y educativos.
"""
# Limpieza de la terminal (Uso como clc en Matlab)
#os.system('cls' if os.name == 'nt' else 'clear')

# Sección de modificación de variables o parámetros globales 
# Configuración de IP
ESP32_CAM_URL = "http://10.238.105.130/stream"  # Cambia por IP de la ESP32-CAM
ESP32_CONTROL_URL = "http://10.238.105.175/control"  # Cambia por IP de la ESP32 controlador
# Variables globales del procesamiento de video
sistema_activo = False
comando_actual = "DETENER"
ultimo_envio = 0
cooldown_comando = 1  # Segundos entre envío de comandos
# Configuración de grabación de video
grabando_video = False
video_writers = {}
# Modificar según condiciones de luz y entorno para la mejor detección

PARAMETROS_COLORES = {
    # H valores de 0-180 en OpenCV
    # S y V valores de 0-255
    'rojo': {
        #Tiene dos rangos para cubrir el espectro circular del rojo
        'hue_min1': 1, 'hue_max1': 10,   
        'hue_min2': 0, 'hue_max2':180 , 
        'sat_min': 5, 'sat_max': 50,  
        'val_min': 50, 'val_max': 200     
    },
    'amarillo': {
        #Un solo rango para amarillo
        'hue_min': 16, 'hue_max': 33,     
        'sat_min': 85, 'sat_max': 255,  
        'val_min': 60, 'val_max': 255    
    },
    'gris': {
        #Un solo rango para gris, saturación baja para gris claro
        'hue_min': 0, 'hue_max': 180,   
        'sat_min': 0, 'sat_max': 5,     
        'val_min': 100, 'val_max': 250
    }
}
# Modificar área mínima para considerar detección válida
AREA_MINIMA_ROJO = 200      
AREA_MINIMA_AMARILLO = 200  
AREA_MINIMA_GRIS = 450   
# Modificar umbrales de intensidad relativa para tomar acciones
# Se dejo de ocupar el amarillo por morivo de nuevo lugar, motivo de umbral alto
UMBRAL_AMARILLO_ALTO = 0.5  
UMBRAL_GRIS_ALTO = 0.1     
# Modificar para cambiar parámetros del preprocesamiento de luz
USAR_FILTROS_LUZ = True     
CLIP_LIMIT = 1.5           # Límite de contraste CLAHE: 1.0-3.0
TILE_GRID_SIZE = 5         # Tamaño de grid : 4-12
# Modificar datos para la detección de personas y cálculo de distancia
#Los datos ingresados en metros
ALTURA_PROMEDIO_PERSONA = 1.75  
ANCHO_PROMEDIO_HOMBROS = 0.45  
DISTANCIA_SEGURIDAD_PERSONAS = 2  
# Configuración de cámara para cálculo de distancia
ANCHO_CAMARA = 800
ALTO_CAMARA = 600
ANGULO_VISION_HORIZONTAL = 60  # ángulo de visión de la cámara
# Umbrales para detección depersonas
CONFIANZA_MINIMA_PERSONA = 0.4  
UMBRAL_AREA_PERSONA = 5000   

# Registro del CVS, para modificar nombre o ruta del archivo
# Modifica los datos a registrar en el CSV en la función registrar_evento
ARCHIVO_CSV = "registro_detecciones.csv" # Nombre del archivo
def inicializar_csv():
    """
    Inicializar archivo CSV con encabezados
    
    En la sección de writerow se pueden agregar o quitar encabezados a guardar
    """
    if not os.path.exists(ARCHIVO_CSV):
        with open(ARCHIVO_CSV, 'w', newline='', encoding='utf-8') as archivo:
            escritor = csv.writer(archivo)
            escritor.writerow([
                'timestamp', 'sistema_activo', 'comando_motores', 
                'rojo_detectado', 'area_rojo', 
                'amarillo_detectado', 'area_amarillo',
                'gris_detectado', 'area_gris',
                'personas_detectadas', 'persona_cercana', 'distancia_minima',
                'accion_tomada'
            ])
        print(f" Archivo CSV creado: {ARCHIVO_CSV}")

def registrar_evento(rojo_detectado, area_rojo, amarillo_detectado, area_amarillo, 
                    gris_detectado, area_gris, personas_detectadas, persona_cercana, 
                    distancia_minima, accion_tomada):
    """
    Registrar evento en archivo CSV con los datos proporcionados
    Tomar en cuenta que al existir ya el archivo cvs, unicamente va agregando la 
    información, no reinicia el archivo. 
    
    Parámetros
    ____________
    rojo_detectado: str
    area_rojo : str
    """
    try:
        with open(ARCHIVO_CSV, 'a', newline='', encoding='utf-8') as archivo:
            escritor = csv.writer(archivo)
            escritor.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                sistema_activo,
                comando_actual,
                rojo_detectado,
                int(area_rojo),
                amarillo_detectado,
                int(area_amarillo),
                gris_detectado,
                int(area_gris),
                personas_detectadas,
                persona_cercana,
                f"{distancia_minima:.2f}" if distancia_minima is not None else "N/A",
                accion_tomada
            ])
    except Exception as e:
        print(f" Error registrando en CSV: {e}")

# Sección que sirve para incializar y detener la grabación de videosm, así como grabar los frame 
def iniciar_grabacion_videos():
    """
    Inicializar grabación de videos
    
    Variables
    ____________
    fps, ANCHO, ALTO: int
        Valor de los fps, y dimensiones (Resolución) al final del video_writers  
        cv2.VideoWriter_fourcc(*'XVID'), fps, (ANCHO, ALTO)),
    video_writers : dict
        Diccionario que almacenará los objetos (VideoWriter) para guardar(escribir) los videos
    grabando_video : bool  
        Boleano indica el estado de la grabación.   
    """
    global video_writers, grabando_video
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_writers = {
        'original': cv2.VideoWriter(f'video_original_{timestamp}.avi', 
                                   cv2.VideoWriter_fourcc(*'XVID'), 10, (800, 600)),
        'procesado': cv2.VideoWriter(f'video_procesado_{timestamp}.avi', 
                                    cv2.VideoWriter_fourcc(*'XVID'), 10, (800, 600))    
    }
    grabando_video = True
    print(" Grabación de videos INICIADA ")

def detener_grabacion_videos():
    """
    Detener grabación de videos y guardar archivos, asi como liberar recursos
    
    Variables
    ____________
    video_writers : dict
        Diccionario que almacenará los objetos (VideoWriter) para escribir los videos
    grabando_video : bool  
        Boleano indica el estado de la grabación.        
    """
    global video_writers, grabando_video
    for nombre, writer in video_writers.items():
        if writer is not None:
            writer.release()
            print(f" Video guardado: {nombre}")
    
    video_writers = {}
    grabando_video = False
    print(" Grabación de videos DETENIDA")

def grabar_frames(frame_original:np.ndarray, frame_procesado:np.ndarray):
    """
    Grabar frames en videos
    
    Parametros
    ____________
    frame_original, frame_procesado : np.ndarray
        Matriz de arreglos de dimensiones (ANCHO_CAMARA,ALTO_CAMARA), conformado
        de areglos para los valores RGB o HSV
    Variables
    ____________
    video_writers : dict
        Diccionario que almacenará los objetos (VideoWriter) para escribir los videos
    grabando_video : bool  
        Boleano indica el estado de la grabación.  
    """
    global video_writers
    if grabando_video and video_writers:
        try:
            if frame_original is not None:
                video_writers['original'].write(frame_original)
            if frame_procesado is not None:
                video_writers['procesado'].write(frame_procesado)
        except Exception as e:
            print(f" Error grabando video: {e}")

# Realiza el preprocesamiento para reducir reflejos y mejorar detección
def aplicar_filtros_luz(frame: np.ndarray) -> np.ndarray:
    """
    Aplicar filtros para reducir reflejos y mejorar detección
    
    Parametros
    ____________
    frame : np.ndarray
        Matriz de arreglos de dimensiones (ANCHO_CAMARA,ALTO_CAMARA), conformado
        de areglos para los valores RGB o HSV
    Variables
    _____________
    CLIP_LIMIT : float
        Para cambiar valor del CLAHE (Limite de la técnica de contraste)
    TILE_GRID_SIZE : int 
        Para cambiar valor del Kernel (Tile) a analizar, recomendable tamaños pequeños 
    Retorno
    _____________
    frame_procesado : np.ndarray
        Frame procesado despues de CLAHE(Mejora de contraste inteligente), y 
        del filtro Gaussiano para reducción de ruido 
    """
    # Convertir a LAB para procesamiento de luminancia
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    # Aplicar CLAHE para mejorar contraste
    clahe = cv2.createCLAHE(clipLimit=CLIP_LIMIT, tileGridSize=(TILE_GRID_SIZE, TILE_GRID_SIZE))
    l = clahe.apply(l)
    # Recombinar canales
    lab = cv2.merge([l, a, b])
    frame_procesado = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    # Aplicar desenfoque para reducir ruido
    frame_procesado = cv2.GaussianBlur(frame_procesado, (5, 5), 0)
    return frame_procesado

# Sección de creación de máscara y detección de color
def crear_mascara_color(frame : np.ndarray, color : str) -> np.ndarray:
    """
    Crear máscara para color específico usando los parámetros configurables
    (Se crea una para cada color).
    
    Parametros
    _____________
    frame : np.ndarray
        Matriz de arreglos de dimensiones (ANCHO_CAMARA,ALTO_CAMARA), conformado
        de areglos para los valores RGB o HSV
    color : str
        Cadena que indica las características de la máscara   
    Variables
    _____________
    PARAMETROS_COLORES : dict [str : dict [str : dict]] 
        Contiene los colores a evaluar, asi como los parámetros de estos en el dict
    Retornos
    _____________
    mascara : np.ndarray/None
        Contiene los datos de la nueva máscara 
    """
    #Verificación de existencia de color, sino retorna nada
    if color not in PARAMETROS_COLORES:
        return None  
    parameters = PARAMETROS_COLORES[color]
    #Conversión de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if color == "rojo":
        bajo_rojo1 = np.array([parameters['hue_min1'], parameters['sat_min'], parameters['val_min']])
        alto_rojo1 = np.array([parameters['hue_max1'], parameters['sat_max'], parameters['val_max']])
        bajo_rojo2 = np.array([parameters['hue_min2'], parameters['sat_min'], parameters['val_min']])
        alto_rojo2 = np.array([parameters['hue_max2'], parameters['sat_max'], parameters['val_max']])  
        #Análisis de los rangos de color, al final union de máscaras con OR 
        mascara1 = cv2.inRange(hsv, bajo_rojo1, alto_rojo1)
        mascara2 = cv2.inRange(hsv, bajo_rojo2, alto_rojo2)
        mascara = cv2.bitwise_or(mascara1, mascara2)
    elif color == "amarillo" or color == "gris":    
        bajo_color = np.array([parameters['hue_min'], parameters['sat_min'], parameters['val_min']])
        alto_color = np.array([parameters['hue_max'], parameters['sat_max'], parameters['val_max']])
        mascara = cv2.inRange(hsv, bajo_color, alto_color)
    # Tamaños de Kernel diferentes para cada color
    kernels = {
        'rojo': 5,      
        'amarillo': 5,  
        'gris': 7       
    }
    #Selección del kernel para las operaciones morfológicas
    #Kernel de tipo cuadrado, tipo de dato unsigned int
    kernel_size = kernels.get(color, 5)
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    # Operaciones morfológicas de CLOSE y OPEN
    # Close: Cierra pequeños huecos dentro de las regiones y une regiones cercanas 
    # Open: Elimina el ruido (pequeñas detecciones aisladas), suaviza contornos. 
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
    return mascara

def detectar_color(frame:np.ndarray, color:str)->Tuple[bool,Optional[float],Optional[np.ndarray]]:
    """
    Detectar color específico en el frame y llama a crear mascara
    
    Parámetros
    ____________
    frame : np.ndarray
        Matriz de arreglos de dimensiones (ANCHO_CAMARA,ALTO_CAMARA), conformado
        de areglos para los valores RGB o HSV
    
    color : str
        Cadena que indica las características de la máscara
    Variables
    ____________
    AREA_MINIMA_* : int 
        Valor para cambiar la sensibilidad del color
    Retorno
    ____________
    Tuple
        detectado : bool 
            Indica si el color fue detectado
        area_total : float/0
            Indica la cantidad total de color en todos los contornos
        mascara : np.ndarray/None
            Contiene los datos de la nueva máscara
    """
    areas_minimas = {
        'rojo': AREA_MINIMA_ROJO,
        'amarillo': AREA_MINIMA_AMARILLO, 
        'gris': AREA_MINIMA_GRIS
    }
    area_minima = areas_minimas.get(color, 500)
    # Llamado a la función crear_mascara_color
    mascara = crear_mascara_color(frame, color)
    if mascara is None:
        return False, 0, None
    # Función para detectar los contornos
    # RETR_EXTERNAL: Solo encuentra los contornos externos de la figura 
    # CHAIN_APPROX_SIMPLE: Elimina puntos redundantes del contorno, puntos críticos
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    detectado = False
    area_total = 0
    # Procesar contornos válidos
    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area > area_minima:
            detectado = True
            area_total += area
    return detectado, area_total, mascara

def calcular_intensidad_color(area_color : float, area_frame: float)->float:
    """
    Calcular intensidad relativa del color en el frame
    
    Parámetros
    ___________
    area_color : float 
        Area total del color calculada a partir de los contornos
    area_frame : float
        Area total del frame 
    """
    if area_frame == 0:
        return 0
    return area_color / area_frame

# Sección de calibrar valores de la camara y calculo de distancia de personas
def calibrar_camara()-> float:
    """
    Calcular parámetros de la cámara para cálculo de distancia
    
    Se parte de calcular la distancia focal, para la estimación de distancias.
    
    Variables
    ___________
    ANCHO_CAMARA : int 
        Valor del ancho de cantidad de pixeles de la camara
    ANGULO_VISION_HORIZONTAL : float 
        Angulo de vision total de ambos lados, se considera (theta)/2 para cada lado
    Retorno : float 
        Distancia focal calculada en pixeles 
    """
    # Calcular distancia focal usando el ángulo de visión
    focal_length = (ANCHO_CAMARA / 2) / math.tan(math.radians(ANGULO_VISION_HORIZONTAL / 2))
    print(f" Cámara calibrada:")
    print(f"   - Focal length: {focal_length:.1f} px")
    print(f"   - Ángulo visión: {ANGULO_VISION_HORIZONTAL}°")
    return focal_length

def calcular_distancia_persona(bbox: list[int,int, int, int], focal_length: float)-> Optional[float]:
    """
    Calcular distancia a persona usando múltiples métodos
    
    Parámetros
    ___________
    bbox: list
        list[x1, y1, x2, y2] lista de puntos críticos
    focal_length : float/None
        Distancia focal previamente calculada en pixeles
    Retorno
    ___________
    float, int
        calculo de distancia final promediada
    """
    #Obtener puntos críticos del rectángulo
    x1, y1, x2, y2 = bbox
    altura_pixeles = y2 - y1
    ancho_pixeles = x2 - x1
    # Primera estimación: Por altura, valores pasan a m
    if altura_pixeles > 50:
        dist_altura = (ALTURA_PROMEDIO_PERSONA * focal_length) / altura_pixeles
    else:
        dist_altura = None
    # Segunda estimación: Por ancho de hombros, valores pasan a m
    if ancho_pixeles > 20:
        dist_ancho = (ANCHO_PROMEDIO_HOMBROS * focal_length) / ancho_pixeles
    else:
        dist_ancho = None
    distancias_validas = []
    #Evaluar si dist_* True y rango de distancia valido
    if dist_altura and 0.3 < dist_altura < 10.0:
        distancias_validas.append(dist_altura)
    if dist_ancho and 0.3 < dist_ancho < 10.0:
        distancias_validas.append(dist_ancho)
    if not distancias_validas:
        return None
    # Promediar las distancias válidas
    return ((sum(distancias_validas) / len(distancias_validas)))/2 # Ajuste empírico (ya es experimental)

# Sección del modelo YOLO para detección de personas y calculo de su distancia
def inicializar_yolo()-> YOLO:
    """
    Inicializar modelo YOLO para detección de personas
    
    Retornos
    ___________
    model : YOLO, None
    """
    try:
        model = YOLO('yolov8n.pt')
        print(" YOLO Ultralytics inicializado correctamente")
        return model
    except Exception as e:
        print(f" Error inicializando YOLO: {e}")
        return None

def detectar_personas_con_distancia(frame: np.ndarray, model: YOLO, focal_length:float)->Tuple[bool, int, bool, Optional[float], np.ndarray]:
    """
    Detectar personas y calcular sus distancias
    
    Parámetros
    ___________
    frame : np.ndarray
        Imagen de la camara
    model : YOLO 
        Modelo de YOLO precargado en el folder
    focal_length : float
        Distancia focal calculada
    Variables
    ____________
    CONFIANZA_MINIMA_PERSONAS : float
        Confianza mínima configurada
    Retornos
    ____________
    Tuple
        personas_detectadas : bool
            Indicador de detector de personas
        num_personas : int
            Cantidad de personas detectadas
        persona_mas_cercana: bool
            Indicador de personas aún más cercanas
        distancia_minima : float/0
            Mínima distancia si existe un conjunto de personas
        frame_anotado : np.ndarray
            Frame modificado con las detecciones de YOLO
    """
    #Verificación de modelo YOLO precargado
    if model is None:
        return False, 0, False, None, frame
    #Ejecuta detección YOLO, verbose->False para suprimir logs de YOLO
    results = model(frame, conf=CONFIANZA_MINIMA_PERSONA, verbose=False)
    personas_detectadas = False
    num_personas = 0
    persona_mas_cercana = False
    distancia_minima = None
    frame_anotado = frame.copy()
    for result in results:
        #Cada box es la detección de una persona 
        boxes = result.boxes
        if boxes is not None:
            for box in boxes:
                #Convertir tensor de PyTorch con el ID de clase a escalar y luego entero, y evaluar num de elementos
                class_id = int(box.cls.item()) if box.cls.numel() > 0 else -1
                #Pasar el tensor de confianza a float
                confidence = box.conf.item()
                # Filtrar por confianza mínima configurable (class_id = 0 -> Persona en COCO) 
                if class_id == 0 and confidence > CONFIANZA_MINIMA_PERSONA:
                    # Obtener coordenadas del bounding box
                    #Obtenemos tensor de coordenadas absolutas, lo pasa a arreglo de numpy y lo pasa a enteros (pixeles)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    # Calcular área y filtrar detecciones pequeñas (elimina falsos positivos)
                    area_persona = (y2 - y1) * (x2 - x1)
                    if area_persona < UMBRAL_AREA_PERSONA:
                        continue
                    num_personas += 1
                    personas_detectadas = True
                    bbox = [x1, y1, x2, y2]
                    #Calculo distancia con calcular_distancia_personas de px a m
                    distancia = calcular_distancia_persona(bbox, focal_length)
                    if distancia is not None:
                        # Actualizar distancia mínima
                        if distancia_minima is None or distancia < distancia_minima:
                            distancia_minima = distancia
                        # Verificar si está dentro de distancia de seguridad
                        if distancia <= DISTANCIA_SEGURIDAD_PERSONAS:
                            persona_mas_cercana = True
                        # Determinar color según distancia
                        if distancia <= DISTANCIA_SEGURIDAD_PERSONAS:
                            color = (0, 0, 255)  # Rojo - muy cercano
                            texto_estado = f'PERSONA: {distancia:.2f}m (CERCA!)'
                        elif distancia <= DISTANCIA_SEGURIDAD_PERSONAS * 2:
                            color = (0, 165, 255)  # Naranja - distancia media
                            texto_estado = f'PERSONA: {distancia:.2f}m'
                        else:
                            color = (0, 255, 0)  # Verde - lejos
                            texto_estado = f'PERSONA: {distancia:.2f}m'
                    else:
                        # Si no se puede calcular distancia, usar color azul
                        color = (255, 0, 0)
                        texto_estado = 'PERSONA: Dist. N/A'
                    # Dibujar bounding box
                    cv2.rectangle(frame_anotado, (x1, y1), (x2, y2), color, 2)
                    # Dibujar texto de información de distancia
                    cv2.putText(frame_anotado, texto_estado, (x1, y1 - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return personas_detectadas, num_personas, persona_mas_cercana, distancia_minima, frame_anotado

# Sección de comunicación con el ESP32 controlador
def enviar_comando_esp32(comando: str, velocidad:int = 0):
    """
    Enviar comando al ESP32 controlador
    
    Parámetros
    ____________
    comando : str
        Comando a enviar (DETENERSE/AVANZAR)
    velocidad : int
        Valor del PWM para los puente H
    Variables
    ____________
    ultimo_comando : float
        Tiempo del ultimo envio enviado
    cooldown_comando : int
        Tiempo de espera entre comandos en seg
    Retorno
    ____________
    bool    
        Indicador de exito de envio
    """
    global ultimo_envio, comando_actual
    if time.time() - ultimo_envio < cooldown_comando:
        return False
    try:
        datos = {'comando': comando, 'velocidad': velocidad}
        #Método POST a URL del ESP32
        respuesta = requests.post(ESP32_CONTROL_URL, data=datos, timeout=1)
        print(f" Comando enviado: {comando}")
        comando_actual = comando
        ultimo_envio = time.time()
        return True
    except Exception as e:
        print(f" Error enviando comando: {e}")
        return False

# Procesamiento completo del video
def procesar_video():
    """
    Función principal que procesa el video stream
    
    Variables
    ____________
    sistema_activo : bool
        Indica el estado del sistema (True permite el envio de comandos al ESP32)
    comando_actual : str
        Comando que se esta enviando al ESP32 
    grabando_video : bool  
        Boleano indica el estado de la grabación. 
    USAR_FILTROS_LUZ : bool 
        Indica el estado del filtro de Luz
    ESP32_CAM_URL : str 
        URL de la ESP32 CAM
    DISTANCIA_SEGURIDAD_PERSONAS : float 
        Valor de la distancia de seguridad en m (no puede avanzar carrito) 
    UMBRAL_AMARILLO_ALTO : float
        Limite de intensidad de amarillo para avanzar
        NT: Alto en el programa porque ya no lo ocupamos
    UMBRAL_GRIS_ALTO : float
        Limite de intensidad de gris para avanzar  
    """
    global sistema_activo, comando_actual, grabando_video
    print(" Iniciando sistema de detección...")
    # Inicializar componentes
    model = inicializar_yolo()
    focal_length = calibrar_camara()  # Calibrar cámara para distancia focal
    inicializar_csv()
    usar_filtros = USAR_FILTROS_LUZ
    #Manejo de errores para evitar codigo quede trabado
    try:
        # Conectar al stream MJPEG del ESP32-CAM
        stream = urlopen(ESP32_CAM_URL, timeout=5)
        bytes_data = b'' #Buffer de datos
        print(" Conectado a ESP32-CAM. Procesando video...")
        print(f" Detección de distancia - Se detiene si persona < {DISTANCIA_SEGURIDAD_PERSONAS}m")
        print("\n CONTROLES:")
        print("   - 'o': Activar sistema")
        print("   - 'v': Iniciar/detener grabación")
        print("   - 'f': Activar/Desactivar filtros de luz") 
        print("   - 'q': Salir")
        # Crear ventanas y darles nombre
        cv2.namedWindow('ESP32-CAM - Sistema Avanzado')
        cv2.namedWindow('Mascara Rojo')
        cv2.namedWindow('Mascara Amarillo')
        cv2.namedWindow('Mascara Gris')
        # Organizar ventanas en la pantalla
        cv2.moveWindow('ESP32-CAM - Sistema Avanzado', 50, 20)
        cv2.moveWindow('Mascara Rojo', 700, 20)
        cv2.moveWindow('Mascara Amarillo', 700, 450)
        cv2.moveWindow('Mascara Gris', 50, 550)
        while True:
            # Leer datos del stream y buscar marcadores JPEG
            bytes_data += stream.read(1024)
            a = bytes_data.find(b'\xff\xd8')  # Inicio JPEG
            b = bytes_data.find(b'\xff\xd9')  # Fin JPEG
            if a != -1 and b != -1:
                jpg = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]
                # Decodificar imagen
                img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    # Redimensionar para rendimiento y obtener frame_original
                    img = cv2.resize(img, (800, 600))
                    frame_original = img.copy()
                    # Aaplicar usar filtros si es requerido
                    if usar_filtros:
                        img_procesada = aplicar_filtros_luz(img)
                    else:
                        img_procesada = img.copy()
                    # Detección de colores y generación de máscaras de color
                    rojo_detectado, area_rojo, mascara_rojo = detectar_color(img_procesada, 'rojo')
                    amarillo_detectado, area_amarillo, mascara_amarillo = detectar_color(img_procesada, 'amarillo')
                    gris_detectado, area_gris, mascara_gris = detectar_color(img_procesada, 'gris')
                    # Calcular intensidades relativas
                    area_frame_total = img_procesada.shape[0] * img_procesada.shape[1]
                    intensidad_amarillo = calcular_intensidad_color(area_amarillo, area_frame_total)
                    intensidad_gris = calcular_intensidad_color(area_gris, area_frame_total)
                    # Detección de personas y cálculo de distancia
                    (personas_detectadas, num_personas, persona_cercana, 
                     distancia_minima, img_anotada) = detectar_personas_con_distancia(img_procesada, model, focal_length)
                    # Sección de visualización de máscaras DE gris a color analizado
                    mascara_rojo_visual = cv2.cvtColor(mascara_rojo, cv2.COLOR_GRAY2BGR) if mascara_rojo is not None else np.zeros((600,800,3), np.uint8)
                    mascara_amarillo_visual = cv2.cvtColor(mascara_amarillo, cv2.COLOR_GRAY2BGR) if mascara_amarillo is not None else np.zeros((600,800,3), np.uint8)
                    mascara_gris_visual = cv2.cvtColor(mascara_gris, cv2.COLOR_GRAY2BGR) if mascara_gris is not None else np.zeros((600,800,3), np.uint8)
                    # Colorear máscaras
                    mascara_rojo_visual[mascara_rojo > 0] = [0, 0, 255] if mascara_rojo is not None else [0,0,0]
                    mascara_amarillo_visual[mascara_amarillo > 0] = [0, 255, 255] if mascara_amarillo is not None else [0,0,0]
                    mascara_gris_visual[mascara_gris > 0] = [128, 128, 128] if mascara_gris is not None else [0,0,0]
                    # Evaluación de acciones
                    accion_tomada = "NINGUNA"
                    if sistema_activo:
                        # PRIORIDAD 1: Persona dentro de distancia de seguridad → DETENER INMEDIATO
                        if persona_cercana:
                            if comando_actual != "DETENER":
                                print(f" DETENER - Persona a {distancia_minima:.2f}m (menos de {DISTANCIA_SEGURIDAD_PERSONAS}m)")
                                if enviar_comando_esp32("DETENER"):
                                    accion_tomada = "DETENER_PERSONA_CERCANA"
                        # PRIORIDAD 2: Mucho gris → DETENER (Ya no tomamos en cuenta el amarillo)
                        elif intensidad_amarillo > UMBRAL_AMARILLO_ALTO or intensidad_gris > UMBRAL_GRIS_ALTO:
                            if comando_actual != "DETENER":
                                print(f" DETENER - Amarillo:{intensidad_amarillo:.2f} Gris:{intensidad_gris:.2f}")
                                if enviar_comando_esp32("DETENER"):
                                    accion_tomada = "DETENER_AMARILLO_GRIS"
                        # PRIORIDAD 3: Rojo detectado y poco gris → AVANZAR  
                        #Valores altos de intensidad para que avance en el caso de gris y amarillo
                        elif rojo_detectado and intensidad_amarillo < 0.7 and intensidad_gris < 0.4:
                            if comando_actual != "AVANZAR":
                                print(" AVANZAR - Rojo detectado")
                                if enviar_comando_esp32("AVANZAR", 120):
                                    accion_tomada = "AVANZAR_ROJO"
                        # PRIORIDAD 4: Ninguna condición → DETENER por seguridad
                        else:
                            if comando_actual != "DETENER":
                                print(" DETENER - Sin condiciones para avanzar")
                                if enviar_comando_esp32("DETENER"):
                                    accion_tomada = "DETENER_SEGURIDAD"
                    # Información crítica del sistema (texto más pequeño)
                    estado_sistema = "ACTIVO" if sistema_activo else "INACTIVO"
                    color_estado = (0, 255, 0) if sistema_activo else (0, 0, 255)
                    # Dibujar información sobre la imagen
                    cv2.putText(img_anotada, f'SISTEMA: {estado_sistema}', (10, 25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_estado, 1)
                    cv2.putText(img_anotada, f'Rojo: {rojo_detectado} ({area_rojo})', (10, 45), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                    cv2.putText(img_anotada, f'Amarillo: {intensidad_amarillo:.2f}', (10, 65), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    cv2.putText(img_anotada, f'Gris: {intensidad_gris:.2f}', (10, 85), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
                    cv2.putText(img_anotada, f'Personas: {num_personas}', (10, 105), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1)
                    if distancia_minima is not None:
                        color_distancia = (0, 0, 255) if persona_cercana else (255, 255, 255)
                        cv2.putText(img_anotada, f'Dist. min: {distancia_minima:.2f}m', (10, 125), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_distancia, 1)
                    cv2.putText(img_anotada, f'Comando: {comando_actual}', (10, 145), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    if accion_tomada != "NINGUNA":
                        cv2.putText(img_anotada, f'ACCION: {accion_tomada}', (10, 170), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    # Registro del CVS
                    #Evalua sistema activo True y accion_tomada diferente a ninguna
                    if sistema_activo and accion_tomada != "NINGUNA":
                        registrar_evento(
                            rojo_detectado, area_rojo, 
                            amarillo_detectado, area_amarillo,
                            gris_detectado, area_gris,
                            num_personas, persona_cercana,
                            distancia_minima if distancia_minima is not None else 0,
                            accion_tomada
                        )
                    # Grabar videos original y procesado
                    if grabando_video:
                        grabar_frames(frame_original, img_anotada)
                    # Mostrar ventanas 
                    cv2.imshow('ESP32-CAM - Sistema Avanzado', img_anotada)
                    cv2.imshow('Mascara Rojo', mascara_rojo_visual)
                    cv2.imshow('Mascara Amarillo', mascara_amarillo_visual) 
                    cv2.imshow('Mascara Gris', mascara_gris_visual)
                    # Manipulación del sistema
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('o'):
                        sistema_activo = True
                        print(" Sistema ACTIVADO")
                        registrar_evento(
                            rojo_detectado, area_rojo, 
                            amarillo_detectado, area_amarillo,
                            gris_detectado, area_gris,
                            num_personas, persona_cercana,
                            distancia_minima if distancia_minima is not None else 0,
                            "SISTEMA_ACTIVADO"
                        )
                    elif key == ord('v'):
                        if not grabando_video:
                            iniciar_grabacion_videos()
                        else:
                            detener_grabacion_videos()
                    elif key == ord('f'):
                        usar_filtros = not usar_filtros
                        estado = "ACTIVADOS" if usar_filtros else "DESACTIVADOS"
                        print(f" Filtros de luz {estado}")
                    elif key == ord('q'):
                        if grabando_video:
                            detener_grabacion_videos()
                        registrar_evento(
                            rojo_detectado, area_rojo, 
                            amarillo_detectado, area_amarillo,
                            gris_detectado, area_gris,
                            num_personas, persona_cercana,
                            distancia_minima if distancia_minima is not None else 0,
                            "SISTEMA_DETENIDO"
                        )
                        break     
    except Exception as e:
        print(f" Error en procesamiento: {e}")
    finally:
        print(" Cerrando aplicación...")
        if grabando_video:
            detener_grabacion_videos()
        enviar_comando_esp32("DETENER")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    print("=" * 60)
    print(" SISTEMA DE DETECCIÓN DE PERSONAS Y PASOS PEATONALES")
    print("=" * 60)
    # Mostrar configuración actual
    print("\n  CONFIGURACIÓN ACTUAL:")
    print(f"   - Umbral Amarillo (detener): {UMBRAL_AMARILLO_ALTO}")
    print(f"   - Umbral Gris (detener): {UMBRAL_GRIS_ALTO}")
    print(f"   - Filtros luz: {'ACTIVADOS' if USAR_FILTROS_LUZ else 'DESACTIVADOS'}")
    print(f"   - Distancia seguridad personas: {DISTANCIA_SEGURIDAD_PERSONAS} m")
    print(f"   - Altura referencia persona: {ALTURA_PROMEDIO_PERSONA} m")
    print(f"   - Ángulo visión cámara: {ANGULO_VISION_HORIZONTAL}°")
    print("\n CONTROLES: o=activar, v=grabar, f=filtros, q=salir")
    print("=" * 60)
    
    procesar_video()