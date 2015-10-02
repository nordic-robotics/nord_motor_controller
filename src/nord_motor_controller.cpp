#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"


/**
 * 
 * Author: Tobias Lundin
 */


class MotorController
{
	public:

	ros::NodeHandle n;

	ros::Publisher PWM_pub;
	ros::Subscriber command_sub;
	ros::Subscriber encoder_sub;

	MotorController(char** argv){

		command_sub = n.subscribe("motor_controller/twist", 1000, &MotorController::commandCallback, this);
		encoder_sub = n.subscribe("aurduino/encoders", 100, &MotorController::encoderCallback, this);
		
		PWM_pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);

		b 	= 20.4; r 	= 9.935;
		pi = 3.141592;

		pwm.PWM1 = 0; pwm.PWM2 = 0;

		p_1 = std::stod(argv[1]); 	p_2 = std::stod(argv[2]);
		i 	= std::stod(argv[3]); 	d 	= std::stod(argv[4]);

		d_p1 = 0; d_p2 = 0;
		i_p1 = 0; i_p2 = 0;

		dt = 1/10; 

		error1 		= 0; error2 	= 0;
		old_error1 	= 0; old_error2 = 0;

	}

	~MotorController()
    {
        delete this;
    }

	void commandCallback(const geometry_msgs::Twist command){
		//forward   = command.linear.x;
		//desired_w = command.angular.z; 
		forward   = 0.2;
		desired_w = 0;
	}

	void encoderCallback(const ras_arduino_msgs::Encoders value){
		estimated_w1 = (value.delta_encoder1)*2*pi*10/(360);
		estimated_w2 = (value.delta_encoder2)*2*pi*10/(360);
		this->controllerPart();
	}


	void  controllerPart(){
		desired_w1 = (forward-(b/2)*desired_w)/r;
		desired_w2 = (forward+(b/2)*desired_w)/r;
		
		error1 = desired_w1 -estimated_w1;
		error2 = desired_w2 -estimated_w2;

		i_p1 += i*(error1)*dt;
		i_p2 += i*(error2)*dt;

		d_p1 = d*(error1-old_error1)/dt;
		d_p2 = d*(error2-old_error2)/dt;

		pwm.PWM1 = p_1*(error1) + i_p1 + d_p1;
 		pwm.PWM2 = p_2*(error2) + i_p2 + d_p2;

 		old_error1 = error1; old_error2= error2;
 		
 		if(pwm.PWM1 > 255){
 			pwm.PWM1 = 255;
 		}
 		else if( pwm.PWM1 < -255){
 			pwm.PWM1 = -255;
 		}
 		if(pwm.PWM2 > 255){
 			pwm.PWM2 = 255;
 		}
 		else if( pwm.PWM2 < -255){
 			pwm.PWM2 = -255;
 		}

 		PWM_pub.publish(pwm);

	}

	void print_info(){
		ROS_INFO("forward: [%f]", forward);
 		ROS_INFO("estimated_w1: [%f]", estimated_w1);
 		ROS_INFO("desired_w1: [%f]", desired_w1);
 		ROS_INFO("estimated_w2: [%f]", estimated_w2);
 		ROS_INFO("desired_w2: [%f]", desired_w2);
 		
		ROS_INFO("i_p1: [%f]", i_p1);
		ROS_INFO("i_p2: [%f]", i_p2);
		double test  = pwm.PWM1;
 		double test2 = pwm.PWM2;
 		ROS_INFO("PWM1: [%f]", test);
 		ROS_INFO("PWM2: [%f]", test2);
	}

	private:

		ras_arduino_msgs::PWM pwm;

		double forward; 	double angle;
		double estimated_w1;double estimated_w2;
		double desired_w1; 	double desired_w2; 
		double desired_w;   double pi;
		double b;			double r;

		double p_1; double p_2;
		double i; double d;

		double error1; 		double error2;
		double old_error1; 	double old_error2; 
		double i_p1; 		double i_p2;
		double d_p1;		double d_p2;

		double dt;
		
 };


int main(int argc, char **argv){

	if(argc<4){
		ROS_INFO("Not enough arguments: p1 p2 i d");
		return 1;
	}
	// Initiates ROS and gives the node name "PWM_MAX"
	ros::init(argc, argv, "motor_controller");
	MotorController run(argv); 
	ros::Rate loop_rate(10);
	

	// ~ while everything is running as it should
	while(ros::ok()){
 		
		// run.controllerPart();
		ros::spinOnce();
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};	


