bool RPMoveBase::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // --- PASO 1: IDENTIFICAR QUÉ ROBOT ESTÁ ACTUANDO ---
        // El parámetro 0 de la acción 'move' es el nombre del robot (ej. "kenny" o "cartman")
        std::string robot_name_pddl = msg->parameters[0].value;
        std::string robot_namespace;

        // --- PASO 2: MAPEAR NOMBRE PDDL A NAMESPACE DE ROS ---
        // Aquí conectamos el nombre lógico (kenny) con el nombre físico (Robot1)
        if (robot_name_pddl == "kenny") {
            robot_namespace = "/Robot1"; 
        } else if (robot_name_pddl == "cartman") {
            robot_namespace = "/Robot2";
        } else {
            // Por seguridad, si llega un nombre raro
            ROS_WARN("Nombre de robot desconocido: %s. Usando Robot1 por defecto.", robot_name_pddl.c_str());
            robot_namespace = "/Robot1";
        }

        // --- PASO 3: CREAR SUBSCRIBER Y PUBLISHER DINÁMICOS ---
        // Construimos los topics usando el namespace seleccionado
        std::string odom_topic = robot_namespace + "/odom";
        std::string cmd_vel_topic = robot_namespace + "/cmd_vel";

        ROS_INFO("Action for robot: %s -> Using topics: %s and %s", robot_name_pddl.c_str(), odom_topic.c_str(), cmd_vel_topic.c_str());

        // Suscribirse a la odometría CORRECTA
        ros::Subscriber sub_odometry = nh_.subscribe(odom_topic, 1, &RPMoveBase::odomCallback, this);
        
        // Publicar en la velocidad CORRECTA
        ros::Publisher velocity_publisher = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);    


        // --- EL RESTO DE TU LÓGICA SIGUE IGUAL ---
        geometry_msgs::Pose2D origen;
        bool s = getCoordenadas(origen, msg, 1); // Indice 1 es ?from
        geometry_msgs::Pose2D destino;
        bool d = getCoordenadas(destino, msg, 2); // Indice 2 es ?to

        if(s && d){
            ROS_INFO_STREAM("SOURCE (" << origen.x << ", " << origen.y << ") TARGET (" << destino.x << ", " << destino.y << ")");

            int movement = classifyMovement(origen, destino);
            ros::Rate rate(10); // He subido un poco el rate para más fluidez
            geometry_msgs::Twist vel_msg;
            float current_distance = 100.0;
            float linear_velocity = 0.2;
            float angular_velocity = 0.2;

            // Loop to move the robot at a specified goal
            // NOTA: Esperamos un poco para recibir el primer mensaje de odometría del nuevo robot
            ros::spinOnce(); 
            ros::Duration(0.2).sleep(); 
            ros::spinOnce();

            while(ros::ok() && current_distance > 0.1){ // Añadido ros::ok() por seguridad
                
                // IMPORTANTE: current_pose se actualiza en odomCallback. 
                // Al cambiar el subscriber arriba, ahora current_pose tendrá los datos del robot correcto.
                
                float inc_x = destino.x - current_pose.x;
                float inc_y = destino.y - current_pose.y;

                float angle_to_goal = atan2(inc_y, inc_x);

                float fine_error = fabs(fabs(angle_to_goal) - fabs(current_pose.theta)); 
                float coarse_error = fabs(angle_to_goal - current_pose.theta);
                
                if(fine_error > 0.1 || coarse_error > 1.5708){
                    vel_msg.linear.x = 0.0;

                    if((angle_to_goal < 0 && current_pose.theta < 0) || (angle_to_goal >= 0 && current_pose.theta >= 0)){
                        if(current_pose.theta > angle_to_goal){
                            vel_msg.angular.z = -angular_velocity;
                        }else{
                            vel_msg.angular.z = angular_velocity;
                        }
                    }else{
                        if(((M_PI - fabs(angle_to_goal)) + (M_PI - fabs(current_pose.theta))) < ((fabs(angle_to_goal) - 0) + (fabs(current_pose.theta) - 0))){
                            if(current_pose.theta > 0) vel_msg.angular.z = angular_velocity;
                            else vel_msg.angular.z = -angular_velocity;
                        }else{
                            if(current_pose.theta > 0) vel_msg.angular.z = -angular_velocity;
                            else vel_msg.angular.z = angular_velocity;
                        }
                    }
                }else{
                    if(movement == 0){ // west
                        if(current_pose.y < destino.y) vel_msg.linear.x = linear_velocity;
                        else vel_msg.linear.x = -linear_velocity;
                    }
                    if(movement == 1){ // north
                        if(current_pose.x < destino.x) vel_msg.linear.x = linear_velocity;
                        else vel_msg.linear.x = -linear_velocity;
                    }
                    if(movement == 2){ // east
                        if(current_pose.y > destino.y) vel_msg.linear.x = linear_velocity;
                        else vel_msg.linear.x = -linear_velocity;
                    }
                    if(movement == 3){ // south
                        if(current_pose.x > destino.x) vel_msg.linear.x = linear_velocity;
                        else vel_msg.linear.x = -linear_velocity;
                    }
                    vel_msg.angular.z = 0.0;
                }

                velocity_publisher.publish(vel_msg);
                ros::spinOnce();
                rate.sleep();
                current_distance= sqrt(pow((current_pose.x - destino.x), 2) + pow((current_pose.y - destino.y), 2));
            }

            // After the loop, stops the robot
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            velocity_publisher.publish(vel_msg);
            ros::Duration(1.0).sleep(); // Usar ros::Duration es mejor que sleep()

            return true;
        }else{
            ROS_FATAL("Invalid format for the waypoints!");
            return false;
        }
    }
