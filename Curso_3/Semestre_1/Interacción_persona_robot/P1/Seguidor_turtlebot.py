#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower_node', anonymous=True)

        # Creamos los Parametros de Control y Comportamiento de nuestro Turtlebot
        self.desired_distance = 0.75
        self.k_linear = 0.5
        self.k_angular = 1.0
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.7
        self.reliability_threshold = 0.7
        self.lost_target_timeout = rospy.Duration(2.0)
        self.control_rate = 10.0

        # Creamos los Parametros para evitar los Obstaculos
        self.obstacle_distance = 0.5  # Distancia en metros para detenerse en funcion al campo de vision
        self.obstacle_fov = 60.0      # Campo de vision frontal en grados (30 a cada lado)

        # Creamos los Parametros de la Rampa de Velocidad para que el turtlebot no de tirones al moverse
        self.linear_acceleration = 0.5 # Cuanto puede aumentar la velocidad lineal por segundo
        self.angular_acceleration = 1.5 # Cuanto puede aumentar la velocidad angular por segundo

        # Creamos el Estado de la Persona a Seguir
        self.target_id = None
        self.last_seen_target = None
        self.last_seen_time = None
        
        # Creamos los Estados de los sensores y control para deter el turtlebot si encuentra obstaculo
        self.obstacle_in_front = False
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

        # Creamos los publicadores y subscriptores para poder detectar y movernos
        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber('people_tracker_measurements', PositionMeasurementArray, self.people_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.loginfo("Nodo seguidor de personas inicializado. Esperando detecciones...")

    # Creamos el callback que permiten detenernos si encontramos algun obstaculo en nuestro campo de vision
    def scan_callback(self, msg):
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        # Calculamos los indices del array que corresponden a nuestra vision frontal
        center_index = num_readings / 2
        fov_rad = math.radians(self.obstacle_fov)
        index_spread = int((fov_rad / 2) / msg.angle_increment)
        
        front_indices = msg.ranges[center_index - index_spread : center_index + index_spread]
        
        self.obstacle_in_front = False
        for distance in front_indices:
            # Comprobamos si la distancia es valida y esta dentro del umbral
            if 0 < distance < self.obstacle_distance:
                self.obstacle_in_front = True
                return # Si encontramos un obstaculo salimos porque no hace falta seguir buscando

    # Creamos la funcion que cada vez que el leg_detector publica nueva informacion, se encarga de obtener y actualizar la persona a seguir
    def people_callback(self, msg):
        current_time = rospy.Time.now()

        # En el caso de que sigamos a alguien ya tenemos un id y buscamos si sigue existiendo la mmisma persona
        if self.target_id:
            target_person = next((p for p in msg.people if p.name == self.target_id and p.reliability > self.reliability_threshold), None)
            # En caso de encontrarlo, actualizamos su posicion y cuando lo hemos visto para poder tener ese timeout
            if target_person:
                self.last_seen_target = target_person
                self.last_seen_time = current_time
        # En caso de no seguir a nadie buscamos a la persona mas cerca que cumpla nuestro umbral de fiabilidad, para poder considerarlo persona
        else:
            closest_person = None
            min_dist = float('inf')
            for person in msg.people:
                if person.reliability > self.reliability_threshold:
                    dist = math.sqrt(person.pos.x**2 + person.pos.y**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_person = person
            # Cuando encontramos a un persona que cumple nuestros parametros, lo establecemos como nuestro nuevo objetivo
            if closest_person:
                self.target_id = closest_person.name
                self.last_seen_target = closest_person
                self.last_seen_time = current_time
                rospy.loginfo("Nuevo objetivo adquirido: {} a {:.2f} metros.".format(self.target_id, min_dist))

    # Creamos nuestro control principal que se ejecuta a 10Hz para no saturar el robot ni la WIFI pero mantener un funcionamiento suave y reactivo
    def run(self):
        rate = rospy.Rate(self.control_rate)
        
        # Calculamos el cambio maximo de velocidad por ciclo de control para nuestra rampa de aceleracion
        linear_acc_step = self.linear_acceleration / self.control_rate
        angular_acc_step = self.angular_acceleration / self.control_rate

        while not rospy.is_shutdown():
            # Mientras no tenemos persona a la cual seguir, el turtlebot no se movera y la velocidad sera 0
            target_linear_speed = 0.0
            target_angular_speed = 0.0

            # En cuanto tenemo una persona a la cual seguit, empezamos a calcular como nos tenemos que mover
            if self.target_id and self.last_seen_target and self.last_seen_time:
                # Calculamos el timeout para saber cuanto tiempo llevamos sin ver al objetivo
                time_since_last_seen = rospy.Time.now() - self.last_seen_time

                # Si el timeout de ver a la persona ha sido excedido, declaramos que la persona no esta y la borramos
                if time_since_last_seen > self.lost_target_timeout:
                    rospy.logwarn("Objetivo {} perdido (timeout). Buscando nuevo objetivo.".format(self.target_id))
                    self.target_id = None
                    self.last_seen_target = None
                else:
                    # Si tenemos una persona a la cual seguir, calculamos las velocidades OBJETIVO para la logica de movimientos
                    x = self.last_seen_target.pos.x
                    y = self.last_seen_target.pos.y
                    distance = math.sqrt(x**2 + y**2)
                    angle_to_target = math.atan2(y, x)

                    # Calculamos la velocidad de giro proporcionalmente al angulo hacia la persona a seguir
                    target_angular_speed = self.k_angular * angle_to_target
                    # Calculamos la velocidad de avance lineal proporcionalmente a la distancia a la persona a seguir
                    target_linear_speed = self.k_linear * (distance - self.desired_distance)

                    # Priorizamos girarnos siempre y cuando esteamos muy cerca de la persona o muy desalineados con la persona a seguir en vez de avanzar
                    if distance < self.desired_distance or abs(angle_to_target) > 0.5:
                        rospy.logwarn_throttle(1.0, "Paro por persona objetivo cerca o hace falta girar")
                        target_linear_speed = 0

            # Aplicamos la Rampa de Aceleracion para poder obtener un funcionamiento suave del turtlebot
            # Mover la velocidad actual hacia la velocidad objetivo, un paso a la vez
            if target_linear_speed > self.current_linear_speed:
                self.current_linear_speed = min(target_linear_speed, self.current_linear_speed + linear_acc_step)
            else:
                self.current_linear_speed = max(target_linear_speed, self.current_linear_speed - linear_acc_step)

            if target_angular_speed > self.current_angular_speed:
                self.current_angular_speed = min(target_angular_speed, self.current_angular_speed + angular_acc_step)
            else:
                self.current_angular_speed = max(target_angular_speed, self.current_angular_speed - angular_acc_step)
            
            # Limitamos las velocidades a los maximos definidos para un funcionamiento suave del turtlebot
            final_linear = max(min(self.current_linear_speed, self.max_linear_speed), -self.max_linear_speed)
            final_angular = max(min(self.current_angular_speed, self.max_angular_speed), -self.max_angular_speed)

            # Aplicamos el Paro de Emergencia por Obstaculo y poniendo la velocidad lineal a 0
            if self.obstacle_in_front and final_linear > 0:
                final_linear = 0.0
                rospy.logwarn_throttle(1.0, "¡Obstáculo frontal detectado! Deteniendo avance.")

            # Publicar la velocidad final que tiene que tener nuestro tutlebot para realizar el seguimiento
            twist_msg = Twist()
            twist_msg.linear.x = final_linear
            twist_msg.angular.z = final_angular
            self.cmd_pub.publish(twist_msg)
            # Loginfo de depuracion para saber a que velocidad y direccion se mueve el turtlebot
            #rospy.loginfo("Velocidad en X: {:2f} y en z {:.2f}".format(final_linear, final_angular))

            # Con esta pausa controlamos que la frecuencia de ejecucion de nuestro codigo es la deseada
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("El tiempo de ROS retrocedió, reiniciando espera.")

# Creamos el main para poder llamar a nuestra clase y que se ejecute el codigo del turtlebot
if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
