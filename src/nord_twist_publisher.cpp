#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"

#include "pid.hpp"
class TwistPublisher
{
	public:

	ros::NodeHandle n;

	ros::Publisher twist_pub;
	ros::Subscriber direction_sub;
	
	TwistPublisher(char ** argv){
		//direction_sub=n.subscribe("name",1000,&TwistPublisher::directionCallback);//NAME OF THE TOPIC!!!!
		twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
		
		twist.linear.x=0; twist.linear.y=0; twist.linear.z=0;
		twist.angular.x=0; twist.angular.y=0; twist.angular.z=0;
		
		est_dist=0; 
		
		p_vel = std::stod(argv[1]); 	p_ang = std::stod(argv[4]);
		i_vel	= std::stod(argv[2]); 	i_ang = std::stod(argv[5]);
		d_vel = std::stod(argv[3]);   d_ang = std::stod(argv[6]);
		
		vel_pid =kontroll::pid<double>(p_vel, i_vel, d_vel);
		vel_pid.max =  3;
		vel_pid.min = -3;
		ang_pid =kontroll::pid<double>(p_ang, i_ang, d_ang);
		ang_pid.max =  45*pi/180;
		ang_pid.min = -45*pi/180;

		pi = 3.141592;
		dt=1.0/10;
	
		for(x=0;x<1000;x++){//testing
			vec_dist[x]=0.2+x*0.1;
			if(x>500){
				vec_degree[x]=25*pi/(180*(x+1));
			}else{
				vec_degree[x]=-25*pi/(180*(x+1));
			}
		}
		
		ControlPart();
	}
	
	void  ControlPart(){
		
		twist.linear.x= vel_pid(est_dist, des_dist, dt);
   		twist.angular.z = ang_pid(est_dir, des_dir, dt);
		
		est_dist +=twist.linear.x*dt;
		est_dir +=twist.angular.z*dt;
		
		twist_pub.publish(twist);
	}
	
	/*void directionCallback(const command){//type of variable?? 
        des_dist= command.distance;
        des_dir= command.degree;
		est_dist=0.2;
		est_dir=0;
	}*/
	
	void updateval(int z){// just for testing
		des_dist= vec_dist[z];
        	des_dir= vec_degree[z];
		est_dist=0.2;
		est_dir=0;
		
	}
	
	void print_info(){
		ROS_INFO("vel: [%f]", twist.linear.x);
 		ROS_INFO("ang_vel: [%f]", twist.angular.z);
 		ROS_INFO("des_dist: [%f]", des_dist);
 		ROS_INFO("est_dist: [%f]", est_dist);
 		ROS_INFO("des_dir: [%f]", des_dir);
		ROS_INFO("est_dir: [%f]", est_dir);
 		ROS_INFO("p_vel: %f i_vel:%f  d_vel:%f ",p_vel,i_vel,d_vel);
 		ROS_INFO("p_ang: %f i_ang:%f  d_ang:%f ",p_ang,i_ang,d_ang);
	}
	
	private:

		geometry_msgs::Twist twist;
		
		double vec_dist[1000]; double vec_degree[1000];
		int x; 
		
		double pi;
		double dt,est_dist,des_dist,est_dir,des_dir;
		
		double p_vel,i_vel,d_vel; double p_ang,i_ang,d_ang;
		kontroll::pid<double> vel_pid; kontroll::pid<double> ang_pid;

	
};
int main(int argc, char **argv){

	if(argc<6){
		ROS_INFO("Not enough arguments: p_vel i_vel d_vel p_ang i_ang d_ang");
		return 1;
	}
	
	ros::init(argc, argv, "nord_twist_publisher");
	
	TwistPublisher run(argv); 
	ros::Rate loop_rate(10);
	
	int z=0;//testing
	int j=30;
	// ~ while everything is running as it should
	while(ros::ok()){
 	
		ros::spinOnce();
		if(j==0){
			z+=1;
			run.updateval(z);
			j=30;
		}else{
			j-=1;
		}
		run.ControlPart();
		run.print_info();
		loop_rate.sleep(); // go to sleep

	}

	return 0;
};
