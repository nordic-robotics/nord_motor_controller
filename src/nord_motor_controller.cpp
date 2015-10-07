#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"

#include "pid.hpp"

class MotorController
{
	public:

	ros::NodeHandle n;

	ros::Publisher PWM_pub;
	ros::Subscriber command_sub;
	ros::Subscriber encoder_sub;

	MotorController(char** argv){

		command_sub = n.subscribe("/motor_controller/twist", 1000, &MotorController::commandCallback, this);
		encoder_sub = n.subscribe("/arduino/encoders", 100, &MotorController::encoderCallback, this);
		
        estimated_w1=0; estimated_w2=0 ;

		PWM_pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);

        	b= 0.204; r= 0.09935;
		pi = 3.141592;

		pwm.PWM1 = 0; pwm.PWM2 = 0;

		p_1 = std::stod(argv[1]); 	p_2 = std::stod(argv[4]);
		i_1	= std::stod(argv[2]); 	i_2 = std::stod(argv[5]);
		d_1 = std::stod(argv[3]);   d_2 = std::stod(argv[6]);
		
		/*p_1=4.9 i_1=2.8 d_1=-0.25
		p_2=6.3 i_2=3.45 d_1=-0.5*/

		d_p1 = 0; d_p2 = 0;
		i_p1 = 0; i_p2 = 0;

		dt = 1.0/10; 

		error1 		= 0; error2 	= 0;
		old_error1 	= 0; old_error2 = 0;

		print_info();

		// pwm1 =kontroll::pid<double>(p_1, i, d);
		// pwm1.max =  255;
		// pwm1.min = -255;
		// pwm2 =kontroll::pid<double>(p_2, i, d);
		// pwm2.max =  255;
		// pwm2.min = -255;

		forward   = 0.6;//delete after gains setting
        	desired_w = 0;
       		desired_w1 = (forward-(b/2)*desired_w)/r;
		desired_w2 = (forward+(b/2)*desired_w)/r;

		controllerPart();

	}

	~MotorController()
    {
        delete this;
    }

	void commandCallback(const geometry_msgs::Twist command){
        forward   = command.linear.x;
        desired_w = command.angular.z;
        desired_w1 = (forward-(b/2)*desired_w)/r;//check signs to turns
		desired_w2 = (forward+(b/2)*desired_w)/r;

	}

	void encoderCallback(const ras_arduino_msgs::Encoders value){
		//delta_1=value.delta_encoder1;
		//delta_2=value.delta_encoder2;
		estimated_w1 = -(value.delta_encoder2)*2*pi*10.0/(360);
		estimated_w2 = -(value.delta_encoder1)*2*pi*10.0/(360);
        //controllerPart();
	}


	void  controllerPart(){
		
		error1 = desired_w1 - estimated_w1;
		error2 = desired_w2 - estimated_w2;

        if (dt!=0){
	        i_p1 += i_1*(error1)*dt;
	        i_p2 += i_2*(error2)*dt;

	        d_p1 = d_1*(error1-old_error1)/dt;
	        d_p2 = d_2*(error2-old_error2)/dt;
        }

        pwm.PWM1 = 20+int(p_1*error1 + i_p1 + d_p1);
        pwm.PWM2 =30+ int(p_2*error2 + i_p2 + d_p2);

       /* if(desired_w1>0 && pwm.PWM1<70){
        	pwm.PWM1=70;
        }
        if(desired_w2>0 && pwm.PWM2<70){
        	pwm.PWM2=70;
        }*/

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
 		// pwm.PWM1 = pwm1(estimated_w1, desired_w1, dt);
   		// pwm.PWM2 = pwm2(estimated_w2, desired_w2, dt);
		ROS_INFO("About to publish");
		print_info();

		// pwm.PWM1 = 100;
		// pwm.PWM2 = 100;
 		PWM_pub.publish(pwm);

	}

	void print_info(){
		ROS_INFO("forward: [%f]", forward);
 		ROS_INFO("estimated_w1: [%f]", estimated_w1);
 		ROS_INFO("desired_w1: [%f]", desired_w1);
 		ROS_INFO("estimated_w2: [%f]", estimated_w2);
 		ROS_INFO("desired_w2: [%f]", desired_w2);
 		ROS_INFO("p1: %f i1:%f  d1:%f ",p_1,i_1,d_1);
 		ROS_INFO("p2: %f i2:%f  d2:%f ",p_2,i_2,d_2);
		ROS_INFO("i_p1: [%f]  i_p2: [%f]", i_p1,i_p2);
		ROS_INFO("d_p1:[%f]   d_p2: [%f]",d_p1,d_p2);
		ROS_INFO("error1:[%f]  error2:[%f]",error1,error2);
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
		double i_1; double i_2;
		double d_1;	double d_2;

		double error1; 		double error2;
		double old_error1; 	double old_error2; 
		double i_p1; 		double i_p2;
		double d_p1;		double d_p2;

		double dt;

		// double delta_1,delta_2;

		//kontroll::pid<double> pwm1; kontroll::pid<double> pwm2;
		
 };


int main(int argc, char **argv){

	if(argc<6){
		ROS_INFO("Not enough arguments: p1 i1 d1 p2 i2 d2");
		return 1;
	}
	// Initiates ROS and gives the node name "PWM_MAX"
	ros::init(argc, argv, "nord_motor_controller");
	
	MotorController run(argv); 
	ros::Rate loop_rate(10);
	

	// ~ while everything is running as it should
	while(ros::ok()){
 		//run.print_info();
      		run.controllerPart();
		ros::spinOnce();
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};


