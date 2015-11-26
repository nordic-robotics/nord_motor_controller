#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "nord_messages/MotorTwist.h"
#include <cmath>
#include "pid.hpp"

class MotorController
{
	public:

	ros::NodeHandle n;
	ros::Publisher PWM_pub;
	ros::Subscriber command_sub;
	ros::Subscriber encoder_sub;

	MotorController(char** argv){

		command_sub = n.subscribe("/nord/motor_controller/twist", 1, &MotorController::commandCallback, this);
		encoder_sub = n.subscribe("/arduino/encoders", 1, &MotorController::encoderCallback, this);
		
        estimated_w1=0; estimated_w2=0 ;

		PWM_pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1);

        b= 0.2015; r= 0.09935/2;
		pi = 3.141592;

		pwm.PWM1 = 0; pwm.PWM2 = 0;

		/*p_1 = std::stod(argv[1]); 	p_2 = std::stod(argv[4]);
		i_1	= std::stod(argv[2]); 	i_2 = std::stod(argv[5]);
		d_1 = std::stod(argv[3]);   d_2 = std::stod(argv[6]);*/

		p_1=0.9; i_1 = 8; d_1=-0.35;
		p_2=0; i_2 = 7.66;  d_2=-0.35;

		/*
		controller with exponentials:
		0.9 8 -0.35 0 7.66 -0.35
		*/

		d_p1 = 0; d_p2 = 0;
		i_p1 = 0; i_p2 = 0;

		dt = 1.0/10; 

		error1 		= 0; error2 	= 0;
		old_error1 	= 0; old_error2 = 0;
		forward   = 0;
		desired_w = 0;

		desired_w1 = (forward+(b/2)*desired_w)/r;//check signs to turns
		desired_w2 = (forward-(b/2)*desired_w)/r;
		print_info();

		
		// pwm1 =kontroll::pid<double>(p_1, i, d);
		// pwm1.max =  255;
		// pwm1.min = -255;
		// pwm2 =kontroll::pid<double>(p_2, i, d);
		// pwm2.max =  255;
		// pwm2.min = -255;



		controllerPart();

	}

	int err_to_pwm1(double err){
		return (61.1*std::exp(err*(0.03917))+1.125*std::exp(err*(0.285)));
	}

	int err_to_pwm2(double err){
		return (58.74*std::exp(err*(0.05823))+0.007478*std::exp(err*(0.5654)));
	}

	void commandCallback(const nord_messages::MotorTwist command){
		//ROS_INFO("ENTER COMMAND CALLBACK");
		forward   = command.velocity;
		desired_w = command.angular_vel;
		desired_w1 = (forward+(b/2)*desired_w)/r;//check signs to turns
		desired_w2 = (forward-(b/2)*desired_w)/r;

	}

	void encoderCallback(const ras_arduino_msgs::Encoders value){
	
		dt=value.timestamp/1000.0;
		estimated_w1 = -(value.delta_encoder2)*2*pi/(360*dt);
		estimated_w2 = -(value.delta_encoder1)*2*pi/(360*dt);
		//ROS_INFO("ENCODER CALLBACK");
        //controllerPart();
	}


	void  controllerPart(){
		
		error1 = desired_w1 - estimated_w1;
		error2 = desired_w2 - estimated_w2;

        if (dt!=0){
	        i_p1 += i_1*(error1)*dt;//*(desired_w1/5.032713)
	        i_p2 += i_2*(error2)*dt;//*(desired_w2/5.032713)

	        d_p1 = d_1*(error1-old_error1)/dt;
	        d_p2 = d_2*(error2-old_error2)/dt;
        }
        if(desired_w1>0){
			pwm.PWM1 = err_to_pwm1(error1)+int(p_1*error1+i_p1 + d_p1);
    	}else if(desired_w1<0){
			pwm.PWM1 = -err_to_pwm1(-error1)+int(p_1*error1+i_p1 + d_p1);
    	}else{
			pwm.PWM1 = 0;
		}
		
    	if(desired_w2>0){
			pwm.PWM2 = err_to_pwm2(error2)+int(p_2*error2+i_p2 + d_p2);
    	}else if(desired_w2<0){
			pwm.PWM2 = -err_to_pwm2(-error2)+int(p_2*error2+i_p2 + d_p2);
    	}else{
			pwm.PWM2 = 0;
		}

 		old_error1 = error1; old_error2= error2;

 		
 		if(pwm.PWM1 > 170){
 			pwm.PWM1 = 170;
 		}
 		else if( pwm.PWM1 < -170){
 			pwm.PWM1 = -170;
 		}
 		if(pwm.PWM2 > 170){
 			pwm.PWM2 = 170;
 		}
 		else if( pwm.PWM2 < -170){
 			pwm.PWM2 = -170;
 		}

		ROS_INFO("About to publish");
		print_info();
		
 		PWM_pub.publish(pwm);

	}

	void print_info(){
		ROS_INFO("forward: [%f] ang_vel: [%f]", forward,desired_w);
 		ROS_INFO("estimated_w1: [%f] estimated_w2: [%f] ", estimated_w1, estimated_w2);
 		ROS_INFO("desired_w1: [%f] desired_w2: [%f]",desired_w1, desired_w2);
 		ROS_INFO("p1:%f i1:%f  d1:%f ",p_1,i_1,d_1);
 		ROS_INFO("p2:%f i2:%f  d2:%f ",p_2,i_2,d_2);
		ROS_INFO("i_p1:[%f]  i_p2:[%f]", i_p1,i_p2);
		ROS_INFO("d_p1:[%f]   d_p2:[%f]",d_p1,d_p2);
		ROS_INFO("error1:[%f]  error2:[%f]",error1,error2);
		double test  = pwm.PWM1;
 		double test2 = pwm.PWM2;
 		ROS_INFO("PWM1:[%f] PWM2:[%f]", test, test2);
	}

	private:

		ras_arduino_msgs::PWM pwm;

		double forward; 	double angle;
		double estimated_w1;double estimated_w2;
		double desired_w1; 	double desired_w2; 
		double desired_w;   double pi;
		double b;			double r;

		double p_1; double p_2;
		double i_1; double i_2;
		double d_1;	double d_2;

		double error1; 		double error2;
		double old_error1; 	double old_error2; 
		double i_p1; 		double i_p2;
		double d_p1;		double d_p2;

		double dt;
		
 };


int main(int argc, char **argv){

	/*if(argc<6){
		ROS_INFO("Not enough arguments: p1 i1 d1 p2 i2 d2");
		return 1;
	}*/
	// Initiates ROS and gives the node name "PWM_MAX"
	ros::init(argc, argv, "nord_motor_controller");
	
	MotorController run(argv); 
	ros::Rate loop_rate(10);
	

	// ~ while everything is running as it should
	while(ros::ok()){
		ros::spinOnce();
      	run.controllerPart();
		
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};


