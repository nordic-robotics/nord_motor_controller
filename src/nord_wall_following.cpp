#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "ras_arduino_msgs/ADConverter.h"
#include "nord_messages/MotorTwist.h"
#include <math.h>


class WallFollowing

{

	public:



	ros::NodeHandle n;



	ros::Publisher twist_pub;

	ros::Subscriber adc_sub;
	int m_val=5;
	int m_val_front=3;

	WallFollowing(char ** argv) : vals_lfront(m_val), vals_lback(m_val),vals_rfront(m_val),vals_rback(m_val),vals_Front(m_val_front){

		adc_sub = n.subscribe("/arduino/adc", 1,&WallFollowing::adcMsgCallback, this);

		twist_pub = n.advertise<nord_messages::MotorTwist>("/nord/motor_controller/twist", 1);

		des_dist=dist_to_adc_short(0.1);
		dist_turn=dist_to_adc_long(0.20);
		
		wait_sensors=m_val;
		
		forward=0.15;
		twist.velocity=forward;
		twist.angular_vel=0; 
		pi=3.14159265359;
		
		Lfront = 0;
  		Lback =0; 
  		Back = 0; 
		Front = 0;
  		Rback = 0; 
		Rfront = 0;

		for(val_i=0;val_i<m_val;val_i+=1){
			vals_lfront[val_i]=0;
			vals_lback[val_i]=0;
			vals_rfront[val_i]=0;
			vals_rback[val_i]=0;
		}
		for(val_i=0;val_i<m_val_front;val_i+=1){
			vals_Front[val_i]=0;
		}
		val_i=0;
		val_i_front=0;

		//g_par=std::stod(argv[1]);
		//g_dist=std::stod(argv[2]);
		
		g_par=0.001;
		g_dist=0;

	}

	int dist_to_adc_short(float x){
		return (276.3*exp(x*(-3.619))+772.3*exp(x*(-26.41)));
	}
	
	int dist_to_adc_long(float x){
		return (793.9*exp(x*(-10.67))+212.5*exp(x*(-1.333)));
	}

	void  ControlPart(){
		vals_lfront[val_i]=Lfront; vals_lback[val_i]=Lback;
		vals_rfront[val_i]=Rfront; vals_rback[val_i]=Rback;
		vals_Front[val_i_front]=Front;
		
		val_i+=1;
		if(val_i==m_val) val_i=0;
		val_i_front+=1;
		if(val_i_front==m_val_front) val_i_front=0;
		
		med_rfront=vecMedian(vals_rfront);
		med_rback=vecMedian(vals_rback);
		med_lfront=vecMedian(vals_lfront);
		med_lback=vecMedian(vals_lback);
		med_Front=vecMedian(vals_Front);
		
		if(wait_sensors==0){
			if(med_Front < dist_turn){
				if ((((med_rfront+med_rback)/2.0)>des_dist) || (((med_lfront+med_lback)/2.0)>des_dist)){
					if(((med_rfront+med_rback)/2.0)<((med_lfront+med_lback)/2.0)){
						twist.angular_vel=(g_par*(med_lfront-med_lback)+g_dist*(des_dist-((med_lfront+med_lback)/2.0)));
					}else{
						twist.angular_vel=-(g_par*(med_rfront-med_rback)+g_dist*(des_dist-((med_rfront+med_rback)/2.0)));
					}
				}else{
					twist.angular_vel=0;
				}
				twist_pub.publish(twist);
			}else{
				ROS_INFO("TURN- dist_turn:[%d] med_Front:[%d]",dist_turn,med_Front);
				if(((Rfront+Rback)/2.0)<((Lfront+Lback)/2.0)){
					twist.angular_vel=pi/2;
				}else{
					twist.angular_vel=-pi/2;
				}
				twist.velocity=0;
				twist_pub.publish(twist);
				ROS_INFO("TURNNNNN");
				ros::Duration(1,100000000).sleep();//Sleep for one second (1,0)1 second and 0 nanoseconds
				twist.angular_vel=0;
				twist.velocity=0;
				twist_pub.publish(twist);
				twist.velocity=forward;
				wait_sensors=m_val;
			}
		}else{
			ROS_INFO("Waiting Sensors: [%d]",wait_sensors);
			wait_sensors-=1;
		}
		

	}

	double vecMedian(std::vector<int> vec){
		if(vec.empty()) return -1;
		else {
			std::sort(vec.begin(), vec.end());
			if(vec.size() % 2 == 0)
					return (vec[vec.size()/2 - 1] + vec[vec.size()/2]) / 2;
			else
					return vec[vec.size()/2];
		}
	}

	void adcMsgCallback(const ras_arduino_msgs::ADConverter adc_values)
	{
  		Lfront = adc_values.ch1;
  		Lback = adc_values.ch3; 
  		Back = adc_values.ch5; 
		Front = adc_values.ch6;
  		Rback = adc_values.ch7; 
		Rfront = adc_values.ch8;
		
		ControlPart();

	}


	void print_info(){

		ROS_INFO("vel: [%f]", twist.velocity);

 		ROS_INFO("ang_vel: [%f]", twist.angular_vel);

 		ROS_INFO("des_dist: [%d]", des_dist);

 		ROS_INFO("med_rfront: [%d]", med_rfront);

 		ROS_INFO("med_rback: [%d]", med_rback);

		ROS_INFO("med_lfront: [%d]", med_lfront);
		ROS_INFO("med_lback: [%d]", med_lback);
		ROS_INFO("med_l: [%f]", ((med_lfront+med_lback)/2.0));
		ROS_INFO("med_r: [%f]", ((med_rfront+med_rback)/2.0));
		ROS_INFO("DIST_TURN: [%d] des_dist: [%d]",dist_turn,des_dist);
		ROS_INFO("med_Front: [%d]",med_Front);
		

	}

	

	private:

		nord_messages::MotorTwist twist; 
		
		int des_dist; int dist_turn;
		unsigned int short Rfront; unsigned int short Rback;
		unsigned int short Lfront; unsigned int short Lback;
		unsigned int short Front; unsigned int short Back;
		
		std::vector<int> vals_rfront; std::vector<int> vals_lfront;
		std::vector<int> vals_rback; std::vector<int> vals_lback;
		std::vector<int> vals_Front;
		int val_i,val_i_front;
		
		int med_rfront; int med_rback;
		int med_lfront; int med_lback;
		int med_Front;
		
		double g_par; double g_dist;
		double forward; double pi;
		
		int wait_sensors;

};

int main(int argc, char **argv){
	
	ros::init(argc, argv, "nord_wall_following");


	WallFollowing run(argv); 

	ros::Rate loop_rate(10);
	

	while(ros::ok()){

		ros::spinOnce();

		run.print_info();

		loop_rate.sleep(); // go to sleep


	}

	return 0;

};
