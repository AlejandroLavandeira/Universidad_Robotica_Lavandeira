#include "ai_planning/RPMoveBase.h"

namespace KCL_rosplan {

    // constructor
    RPMoveBase::RPMoveBase(ros::NodeHandle &nh) {
        nh_ = nh;
    }
    

    bool RPMoveBase::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        ros::Subscriber sub_odometry = nh_.subscribe("/Robot1/odom", 1, &RPMoveBase::odomCallback, this);
        ros::Publisher velocity_publisher = nh_.advertise<geometry_msgs::Twist>("/Robot1/cmd_vel",1);    
        ros::Publisher velocity_publisher = nh_.advertise<geometry_msgs::Twist>("/Robot2/cmd_vel",1);    

        geometry_msgs::Pose2D origen;
        bool s = getCoordenadas(origen, msg, 1);
        geometry_msgs::Pose2D destino;
        bool d = getCoordenadas(destino, msg, 2);
        if(s && d){
            ROS_INFO_STREAM("SOURCE (" << origen.x << ", " << origen.y << ") TARGET (" << destino.x << ", " << destino.y << ")");

            int movement = classifyMovement(origen, destino);
            ros::Rate rate(3);
            geometry_msgs::Twist vel_msg;
            float current_distance = 100.0;
            float linear_velocity = 0.2;
            float angular_velocity = 0.2;
            // Loop to move the robot at a specified goal
            while(current_distance > 0.1){
                float inc_x = destino.x - current_pose.x;
                float inc_y = destino.y - current_pose.y;

                float angle_to_goal = atan2(inc_y, inc_x);

                float fine_error = fabs(fabs(angle_to_goal) - fabs(current_pose.theta)); // Check angle with goal
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
                //ROS_INFO_STREAM(" POSE " << current_pose.x << " " << current_pose.y << " " << destino.x << " " << destino.y << " " << current_distance << " " << fine_error);
            }

            // After the loop, stops the robot
            vel_msg.angular.z = 0;
            vel_msg.linear.x = 0;
            velocity_publisher.publish(vel_msg);
            sleep(1);

            return true;
        }else{
            ROS_FATAL("Invalid format for the waypoints! The correct format in the pddl problem is wpxy where xy is the position of the waypoint. For example, wp00, wp01, wp23, ...");

            return false;
        }
    }

    void RPMoveBase::odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        // linear position
        current_pose.x = msg->pose.pose.position.x;
        current_pose.y = msg->pose.pose.position.y;

        // quaternion to RPY conversion
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // angular position
        current_pose.theta = yaw;
    }

    bool RPMoveBase::getCoordenadas(geometry_msgs::Pose2D& point, const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg, int index){
        if((msg->parameters[index].value).rfind("wp", 0) == 0) {
            if(index == 1) ROS_INFO_STREAM("SOURCE WAYPOINT " << (msg->parameters[index].value));
            else if(index == 2) ROS_INFO_STREAM("TARGET WAYPOINT " << (msg->parameters[index].value));
            try{
                point.x = (msg->parameters[index].value).at(2) - '0';
                point.y = (msg->parameters[index].value).at(3) - '0';
            }catch(const std::out_of_range& e){
                return false;
            }
        }else{
            return false;
        }
        return true;
    }

    int RPMoveBase::classifyMovement(geometry_msgs::Pose2D origen, geometry_msgs::Pose2D destino){
        int movement = -1;
        if(origen.x < destino.x){
            movement = 1; // north
        }else if(origen.x > destino.x){
            movement = 3; // south
        }else if(origen.y > destino.y){
            movement = 2; // east
        }else if(origen.y < destino.y){
            movement = 0; // west
        }
        return movement;
    }

    
} // close namespace

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_movebase");

    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    KCL_rosplan::RPMoveBase rpmb(nh);

    rpmb.runActionInterface();

    return 0;
}
