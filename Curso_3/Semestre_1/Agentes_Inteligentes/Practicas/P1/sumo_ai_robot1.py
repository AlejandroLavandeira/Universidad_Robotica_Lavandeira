#!/usr/bin/env python2
import rospy
import sys
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class SumoAI:
    def __init__(self, ns):
        self.ns = ns
        rospy.loginfo("Iniciando el nodo de IA para el robot: %s", self.ns)

        # --- 1. Definicion de la Maquina de Estados ---
        self.ESTADO_BUSCANDO = "BUSCANDO"
        self.ESTADO_FLANQUEANDO = "FLANQUEANDO"
        self.ESTADO_EMPUJANDO = "EMPUJANDO"
        self.ESTADO_MANIOBRA_DEFENSIVA = "MANIOBRA_DEFENSIVA"
        self.ESTADO_EVITANDO_BORDE = "EVITANDO_BORDE"

        # Estado inicial
        self.estado_actual = self.ESTADO_BUSCANDO
        self.estado_previo = None # Util para depuracion
		self.game_over = False

        # --- 2. Parametros y Umbrales ---
        # Parametros del Dohyo y Seguridad
        self.RADIO_DOHYO = 2.0
        self.DISTANCIA_SEGURIDAD_BORDE = 1.8 # Empezamos a evitar el borde a 1.8m del centro

        # Parametros de Combate
        self.UMBRAL_DETECCION_ENEMIGO = 3.0 # Consideramos enemigo a cualquier objeto a menos de 3m
        self.UMBRAL_DE_EMPUJE = 0.8 # Distancia a la que pasamos de flanquear a empujar (40cm)
        self.UMBRAL_PERDIDA_EMPUJE = -0.01 # Si nuestra velocidad lineal.x es negativa, nos ganan

		self.maniobra_start_time = None
		self.DURACION_MANIOBRA_DEFENSIVA = 1.5
        # Parametros de Movimiento del Robot
        self.VELOCIDAD_LINEAL_MAX = 0.5
        self.VELOCIDAD_ANGULAR_MAX = 0.8

        # --- 3. Almacenes de Datos de Sensores ---
        self.datos_laser = None
        self.posicion_actual = None
        self.velocidad_actual = None

        # --- 4. Publicadores y Suscriptores de ROS ---
        # Publicador para enviar comandos de velocidad
        self.pub_cmd_vel = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=1)

        # Suscriptor a los datos del laser
        rospy.Subscriber(self.ns + '/scan', LaserScan, self.callback_laser)

        # Suscriptor a los datos de odometria
        rospy.Subscriber(self.ns + '/odom', Odometry, self.callback_odometria)

	# Suscriptor a los datos de winner
        rospy.Subscriber("/sumo/winner", String, self.winner_callback)

        try:
            rospy.sleep(4) # Pausa de 4 segundos sincronizada con el reloj de ROS/Gazebo
        except rospy.ROSInterruptException:
            return # Salimos si el nodo se cierra durante la pausa
		
        # --- 5. El "Corazon" de la IA: un temporizador que ejecuta la logica 10 veces por segundo ---
        rospy.Timer(rospy.Duration(0.1), self.ejecutar_ciclo)

    # --- Callbacks (se ejecutan cuando llega un mensaje de un sensor) ---
    def callback_laser(self, msg):
        self.datos_laser = msg

    def callback_odometria(self, msg):
        # Extraemos solo la informacion que nos interesa del mensaje de odometria
        self.posicion_actual = msg.pose.pose.position
        self.velocidad_actual = msg.twist.twist.linear

    # --- Metodos de Accion (crean y publican mensajes Twist) ---
    def mover(self, vel_lineal_x, vel_angular_z):
        """Metodo centralizado para publicar el movimiento."""
        twist_msg = Twist()
        twist_msg.linear.x = vel_lineal_x
        twist_msg.angular.z = vel_angular_z
        self.pub_cmd_vel.publish(twist_msg)

    # --- Metodos de Condicion (devuelven True o False para las transiciones) ---
    def detecto_borde(self):
        if self.posicion_actual is None:
            return False
        # Calculamos la distancia euclidiana desde el centro (0,0)
        distancia_al_centro = math.sqrt(self.posicion_actual.x**2 + self.posicion_actual.y**2)
        return distancia_al_centro > self.DISTANCIA_SEGURIDAD_BORDE

    def analizar_laser(self):
        """Analiza el laser y devuelve la distancia minima y el angulo del enemigo."""
        if self.datos_laser is None:
            return (False, 0, 0) # No hay enemigo, distancia 0, angulo 0

        min_dist = self.UMBRAL_DETECCION_ENEMIGO
        indice_min = -1

        # Buscamos la distancia mas corta en el array de rangos
        for i, dist in enumerate(self.datos_laser.ranges):
            if 0 < dist < min_dist:
                min_dist = dist
                indice_min = i

        if indice_min == -1:
            return (False, 0, 0) # No se detecto nada dentro del umbral
        else:
            # Calculamos el angulo correspondiente a la distancia minima
            angulo_enemigo = self.datos_laser.angle_min + indice_min * self.datos_laser.angle_increment
            return (True, min_dist, angulo_enemigo)

    def me_estan_ganando(self):
        if self.velocidad_actual is None:
            return False
		elif self.velocidad_actual.x < self.UMBRAL_PERDIDA_EMPUJE:
			rospy.loginfo("La velocidad actual es menor a nuestro umbral, estamos yendo hacia atras")
        # Si el enemigo esta muy cerca y nuestra velocidad es negativa, estamos perdiendo.
        return self.velocidad_actual.x < self.UMBRAL_PERDIDA_EMPUJE

    def winner_callback(self, msg):
		if self.game_over:
			return
		self.game_over = True
		self.mover(0,0)
		rospy.signal_shutdown("Combate finalizado")

    # --- El Bucle Principal de la Maquina de Estados ---
    def ejecutar_ciclo(self, event):
        # No hacemos nada hasta que no tengamos datos de los sensores
        if self.datos_laser is None or self.posicion_actual is None or self.velocidad_actual is None:
            rospy.loginfo("Esperando datos de sensores para %s...", self.ns)
            return

        # Para depurar, imprimimos el estado solo cuando cambia
        if self.estado_actual != self.estado_previo:
            rospy.loginfo("Robot %s: Cambiando de estado de %s a %s", self.ns, self.estado_previo, self.estado_actual)
            self.estado_previo = self.estado_actual

        # Analizamos el laser una vez por ciclo
        (enemigo_detectado, dist_enemigo, angulo_enemigo) = self.analizar_laser()
        enemigo_muy_cerca = enemigo_detectado and dist_enemigo < self.UMBRAL_DE_EMPUJE

		if enemigo_detectado:
            # Convertimos el angulo a grados para que sea mas facil de leer
            angulo_grados = math.degrees(angulo_enemigo)
            rospy.loginfo("[%s] Estado: %s | Enemigo DETECTADO a %.2f m y %.1f deg", self.ns, self.estado_actual, dist_enemigo, angulo_grados)
        else:
            rospy.loginfo("[%s] Estado: %s | No se detecta enemigo.", self.ns, self.estado_actual)

		if self.game_over:
			return
			
        # --- LOGICA DE TRANSICIONES (CON PRIORIDAD) ---
        # 1. La condicion de maxima prioridad: evitar el borde
        if self.detecto_borde():
            self.estado_actual = self.ESTADO_EVITANDO_BORDE
        # 2. Logica para el resto de estados
        elif self.estado_actual == self.ESTADO_BUSCANDO:
            if enemigo_detectado:
                self.estado_actual = self.ESTADO_FLANQUEANDO
        
        elif self.estado_actual == self.ESTADO_FLANQUEANDO:
            if not enemigo_detectado:
                self.estado_actual = self.ESTADO_BUSCANDO
            elif enemigo_muy_cerca:
                self.estado_actual = self.ESTADO_EMPUJANDO

        elif self.estado_actual == self.ESTADO_EMPUJANDO:
            if not enemigo_detectado:
                self.estado_actual = self.ESTADO_BUSCANDO
            elif self.me_estan_ganando():
				rospy.loginfo("Entramos en zona defensiva")
                self.estado_actual = self.ESTADO_MANIOBRA_DEFENSIVA
				self.maniobra_start_time = rospy.Time.now()
        
        elif self.estado_actual == self.ESTADO_MANIOBRA_DEFENSIVA:
            # Esta es la condicion de salida
            if self.maniobra_start_time is not None:
                duracion_transcurrida = (rospy.Time.now() - self.maniobra_start_time).to_sec()
                if duracion_transcurrida > self.DURACION_MANIOBRA_DEFENSIVA:
                    self.estado_actual = self.ESTADO_BUSCANDO
                    self.maniobra_start_time = None # Reseteamos el timer para la proxima vez

        elif self.estado_actual == self.ESTADO_EVITANDO_BORDE:
            # Si ya no detectamos el borde, volvemos a buscar
            if not self.detecto_borde():
                self.estado_actual = self.ESTADO_BUSCANDO


        # --- LOGICA DE ACCIONES (basada en el estado actual) ---
        if self.estado_actual == self.ESTADO_BUSCANDO:
            self.mover(0, self.VELOCIDAD_ANGULAR_MAX * 0.6) # Girar a velocidad media

        elif self.estado_actual == self.ESTADO_FLANQUEANDO:
            # Avanzamos y giramos a la vez para movernos en arco hacia el enemigo
            vel_lineal = self.VELOCIDAD_LINEAL_MAX * 0.7
            # La velocidad angular es proporcional al angulo del enemigo para corregir la trayectoria
            vel_angular = angulo_enemigo * 1.5 
            self.mover(vel_lineal, vel_angular)

        elif self.estado_actual == self.ESTADO_EMPUJANDO:
            # Maxima potencia hacia adelante, con pequenas correcciones para mantenerlo centrado
            vel_angular = angulo_enemigo * 0.5
            self.mover(self.VELOCIDAD_LINEAL_MAX, vel_angular)

        elif self.estado_actual == self.ESTADO_MANIOBRA_DEFENSIVA:
            # Retrocedemos y giramos bruscamente para escapar
            self.mover(self.VELOCIDAD_LINEAL_MAX * 0.3, -self.VELOCIDAD_ANGULAR_MAX)

        elif self.estado_actual == self.ESTADO_EVITANDO_BORDE:
            # Retrocedemos para alejarnos del borde. El giro depende de nuestra posicion.
            # Angulo hacia el centro del dohyo
            angulo_hacia_el_centro = math.atan2(-self.posicion_actual.y, -self.posicion_actual.x)
            # Por simplicidad, retrocedemos y giramos hacia el centro
            self.mover(-self.VELOCIDAD_LINEAL_MAX * 0.8, angulo_hacia_el_centro * 0.5)


if __name__ == '__main__':
    try:
        rospy.init_node('sumo_ai')
        # El namespace del robot se pasa como argumento de linea de comandos
        # o se lee de un parametro de ROS. Esto es estandar en ROS.
        ns = sys.argv[1] if len(sys.argv) > 1 else rospy.get_param('~robot_ns', 'robot1')
        SumoAI(ns)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
