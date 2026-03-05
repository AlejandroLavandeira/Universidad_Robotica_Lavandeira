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
        rospy.loginfo("Iniciando el nodo de IA para el robot: %s (Estrategia: Bulldozer Agresivo)", self.ns)

        # --- 1. Definicion de la Maquina de Estados (Estrategia Diferente) ---
        self.ESTADO_PATRULLANDO = "PATRULLANDO"
        self.ESTADO_CARGANDO = "CARGANDO"
        self.ESTADO_EVITANDO_BORDE = "EVITANDO_BORDE"

        # Estado inicial
        self.estado_actual = self.ESTADO_PATRULLANDO
        self.estado_previo = None
		self.game_over = False

        # --- 2. Parametros y Umbrales (pueden ser los mismos que robot1) ---
        self.RADIO_DOHYO = 2.0
        self.DISTANCIA_SEGURIDAD_BORDE = 1.8
        self.UMBRAL_DETECCION_ENEMIGO = 4.0 # Un poco mas 'alerta'
        
        self.VELOCIDAD_LINEAL_MAX = 0.5
        self.VELOCIDAD_ANGULAR_MAX = 0.8

        # --- 3. Almacenes de Datos de Sensores ---
        self.datos_laser = None
        self.posicion_actual = None

        # --- 4. Publicadores y Suscriptores de ROS ---
        self.pub_cmd_vel = rospy.Publisher(self.ns + '/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber(self.ns + '/scan', LaserScan, self.callback_laser)
        rospy.Subscriber(self.ns + '/odom', Odometry, self.callback_odometria)
		rospy.Subscriber("/sumo/winner", String, self.winner_callback)

        try:
            rospy.sleep(4) # Pausa de 4 segundos sincronizada con el reloj de ROS/Gazebo
        except rospy.ROSInterruptException:
            return # Salimos si el nodo se cierra durante la pausa

        # --- 5. El "Corazon" de la IA ---
        rospy.Timer(rospy.Duration(0.1), self.ejecutar_ciclo)

    # --- Callbacks (Reutilizados, son iguales que en robot1) ---
    def callback_laser(self, msg):
        self.datos_laser = msg

    def callback_odometria(self, msg):
        self.posicion_actual = msg.pose.pose.position

    # --- Metodos de Accion ---
    def mover(self, vel_lineal_x, vel_angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = vel_lineal_x
        twist_msg.angular.z = vel_angular_z
        self.pub_cmd_vel.publish(twist_msg)

    # --- Metodos de Condicion (Reutilizados) ---
    def detecto_borde(self):
        if self.posicion_actual is None:
            return False
        distancia_al_centro = math.sqrt(self.posicion_actual.x**2 + self.posicion_actual.y**2)
        return distancia_al_centro > self.DISTANCIA_SEGURIDAD_BORDE

    def analizar_laser(self):
        if self.datos_laser is None:
            return (False, 0, 0)

        min_dist = self.UMBRAL_DETECCION_ENEMIGO
        indice_min = -1
        for i, dist in enumerate(self.datos_laser.ranges):
            if 0 < dist < min_dist:
                min_dist = dist
                indice_min = i

        if indice_min == -1:
            return (False, 0, 0)
        else:
            angulo_enemigo = self.datos_laser.angle_min + indice_min * self.datos_laser.angle_increment
            return (True, min_dist, angulo_enemigo)

    def winner_callback(self,msg):
		if self.game_over:
			return
		self.game_over=True
		self.mover(0,0)
		rospy.signal_shutdown("El combate ha terminado")

    # --- El Bucle Principal de la Maquina de Estados (LOGICA DIFERENTE) ---
    def ejecutar_ciclo(self, event):
        if self.datos_laser is None or self.posicion_actual is None:
            return

        if self.estado_actual != self.estado_previo:
            rospy.loginfo("Robot %s: Cambiando de estado de %s a %s", self.ns, self.estado_previo, self.estado_actual)
            self.estado_previo = self.estado_actual

        (enemigo_detectado, dist_enemigo, angulo_enemigo) = self.analizar_laser()

        #if enemigo_detectado:
        #    angulo_grados = math.degrees(angulo_enemigo)
        #    rospy.loginfo("[%s] Estado: %s | Enemigo DETECTADO a %.2f m y %.1f deg", self.ns, self.estado_actual, dist_enemigo, angulo_grados)
        #else:
        #    rospy.loginfo("[%s] Estado: %s | No se detecta enemigo.", self.ns, self.estado_actual)

		if self.game_over:
			return

        # --- LOGICA DE TRANSICIONES (Mas simple y directa) ---
        if self.detecto_borde():
            self.estado_actual = self.ESTADO_EVITANDO_BORDE
        
        elif self.estado_actual == self.ESTADO_PATRULLANDO:
            if enemigo_detectado:
                self.estado_actual = self.ESTADO_CARGANDO
        
        elif self.estado_actual == self.ESTADO_CARGANDO:
            if not enemigo_detectado:
                self.estado_actual = self.ESTADO_PATRULLANDO

        elif self.estado_actual == self.ESTADO_EVITANDO_BORDE:
            if not self.detecto_borde():
                self.estado_actual = self.ESTADO_PATRULLANDO

        # --- LOGICA DE ACCIONES (Comportamiento Agresivo) ---
        if self.estado_actual == self.ESTADO_PATRULLANDO:
            # En lugar de girar en el sitio, se mueve en un arco amplio.
            # Es una busqueda activa y agresiva.
            self.mover(self.VELOCIDAD_LINEAL_MAX * 0.4, self.VELOCIDAD_ANGULAR_MAX * 0.7)

        elif self.estado_actual == self.ESTADO_CARGANDO:
            # Carga directa hacia el enemigo. Maxima velocidad lineal, solo corrige angulo.
            self.mover(self.VELOCIDAD_LINEAL_MAX, angulo_enemigo * 1.2)

        elif self.estado_actual == self.ESTADO_EVITANDO_BORDE:
            # La misma logica de supervivencia que robot1.
            self.mover(-self.VELOCIDAD_LINEAL_MAX, self.VELOCIDAD_ANGULAR_MAX)


if __name__ == '__main__':
    try:
        rospy.init_node('sumo_ai_robot2')
        ns = sys.argv[1] if len(sys.argv) > 1 else rospy.get_param('~robot_ns', 'robot2')
        SumoAI(ns)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
