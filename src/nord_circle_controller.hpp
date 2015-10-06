#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv){
	// Initiates ROS and gives the node name "PWM_MAX"
	ros::init(argc, argv, "nord_circle_controller");
	
	ros::NodeHandle n; 

    ros::Publisher speed_pub = n.advertise<geometry_msgs::Twist>("motor_controller/twist", 1000);

	ros::Rate loop_rate(10);

	double pi = 3.141592;
	double r = 0.50;
	double the_time = 10.0;

	geometry_msgs::Twist speed;	

	while(ros::ok()){
 		
        speed.linear.x = (2*pi*r*2)/the_time; // distance/s = (2*pi*d)/the_time
		speed.angular.z = (2*pi)/the_time; // radians/s
		speed_pub.publish(speed);
		ros::spinOnce(); //added but not really needed
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};