import cv2

# Funcion para calcular el centro de la persona y el tamano del rectangulo de la persona
def hallar_centro_tam(top_left, bottom_right):

    # Suma las esquinas superiores izquierdas con las esquinas inferiores derechas y divide la suma entre 2, para luego guardarlo en una tupla
    cx = int((top_left[0] + bottom_right[0])/2) 
    cy = int((top_left[1] + bottom_right[1])/2)
    pos = (cx, cy)  # El centro
    
    tam = int(bottom_right[0]-top_left[0])  # Resta la esquina superior izquierda y la esquina inferior derecha para conseguir el tamano

    return pos,tam

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Deteccion Video
cap = cv2.VideoCapture(0)

# Bucle para procesamiento de cada frame
if not cap.isOpened():
    print("No se pudo abrir la camara")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Redimensionar para mejorar rendimiento
    frame = cv2.resize(frame, (640, 480))

    # Deteccion de personas
    rects, weights = hog.detectMultiScale(frame,
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

    cv2.rectangle(frame, (ptos[0], ptos[1]), (ptos[0] + ptos[2], ptos[1] + ptos[3]), (0, 255, 0), 2)
    cv2.circle(frame, zen, 10, (0,0,255),-1)  # Dibuja el centro de color rojo

    # Mostrar el video con las detecciones
    cv2.imshow("Deteccion de personas (HOG)", frame)

    # Salir si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera la captura y cierra todas las ventanas de OpenCV
cap.release()
cv2.destroyAllWindows()
