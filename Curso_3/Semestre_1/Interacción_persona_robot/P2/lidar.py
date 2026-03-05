#!/usr/bin/env python3

# Importa las librerias necesarias 
import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
import math

rospy.init_node("Nodo") # Crea el nodo

dist=0
frontal_x=0
prof=0
prof_img=np.zeros((480,640),dtype=np.uint16)
puente = CvBridge()  
imagen = Image()

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Filtro de Kalman 2D: estado = [x, y, dx, dy]
kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)
kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1

# Funcion para calcular el centro de la persona y el tamano del rectangulo de la persona
def hallar_centro_tam(top_left, bottom_right):

    # Suma las esquinas superiores izquierdas con las esquinas inferiores derechas y divide la suma entre 2, para luego guardarlo en una tupla
    cx = int((top_left[0] + bottom_right[0])/2) 
    cy = int((top_left[1] + bottom_right[1])/2)
    pos = (cx, cy)  # El centro
    
    tam = int(bottom_right[0]-top_left[0])  # Resta la esquina superior izquierda y la esquina inferior derecha para conseguir el tamano

    return pos,tam

def deteccion(frame):
    

    # Redimensionar para mejorar rendimiento
    img = cv2.resize(frame, (640, 480))

    # Deteccion de personas
    rects, weights = hog.detectMultiScale(img,
                                          winStride=(4, 4),
                                          padding=(8, 8),
                                          scale=1.2)
    elegido = 0
    zen = (0,0)
    ptos = (0,0,0,0)
    # Dibujar rectangulos
    for (x, y, w, h) in rects:
        cent, tam = hallar_centro_tam((x,y),(x+w, y+h))
        #print(tam)
        if tam > elegido:
            elegido = tam
            zen = cent
            ptos = (x, y, w, h)

    # Prediccion del filtro
    prediccion = kalman.predict()
    pred_x, pred_y = int(prediccion[0]), int(prediccion[1])

    if zen != (0,0):
        # Correccion con medicion real
        medicion = np.array([[np.float32(zen[0])],
                             [np.float32(zen[1])]])
        kalman.correct(medicion)

        return elegido, zen, ptos

        # Dibujar deteccion real
        # cv2.rectangle(frame, (ptos[0], ptos[1]), (ptos[0] + ptos[2], ptos[1] + ptos[3]), (0, 255, 0), 2)
        # cv2.circle(frame, zen, 10, (0, 0, 255), -1)
    else:
        # No hay deteccion, se usa la prediccion
        #cv2.putText(frame, "Prediccion activa (sin deteccion)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cent, tam = hallar_centro_tam((pred_x,pred_y),(pred_x+150, pred_y+200))

        return tam, cent, (pred_x,pred_y,100,150)

# Callback del rplidar
# Callback de la camara
def cb_camera(msg):
    global imagen
     
    imag = puente.imgmsg_to_cv2(msg)  # Convierte el mensaje

    imagen=cv2.cvtColor(imag,cv2.COLOR_BGR2RGB) # Invierte los colores de la imagen

    height, width, _ = imagen.shape # Coge la altura y el ancho de la imagen

    imagen = cv2.resize(imagen, ( int(height*1.5), int(width))) # Redimensiona la imagen 

def cb_d(msg):

    global prof_img
    
    prof_img=puente.imgmsg_to_cv2(msg,desired_encoding='passthrough')





rospy.Subscriber("/robot/front_rgbd_camera/depth/image_raw",Image,cb_d)
rospy.Subscriber("/robot/front_rgbd_camera/rgb/image_raw",Image,cb_camera)
rate = rospy.Rate(4)
time.sleep(3)
# Subscriptores
while not rospy.is_shutdown():

    if (cv2.waitKey(1) == ord('s')):
        break

    #print(len(sub.ranges))

    elegido, zen, ptos = deteccion(imagen)  # Llama a la funcion para detectar a la persona

    top_left = (ptos[0], ptos[1])
    bottom_right = (ptos[0] + ptos[2], ptos[1] + ptos[3])

    cv2.rectangle(imagen, top_left, bottom_right, (0, 255, 0), 2)
    cv2.circle(imagen, zen, 10, (0,0,255),-1)  # Dibuja el centro de color rojo

    prof_reg=prof_img[zen[1]-2:zen[1]+3,zen[0]-2:zen[0]+3]
    prof_reg= prof_reg[prof_reg>0]

    if prof_reg.size > 0:

        prof = np.mean(prof_reg)

    else:

        print("No hay valores en la region")

    if prof > 10:
        prof = prof / 1000.0

    print(prof)

    cv2.imshow('webCam',imagen) # Muestra la imagen por pantalla

    
    rate.sleep()

cv2.destroyAllWindows()  # Destruye todas las ventanas
