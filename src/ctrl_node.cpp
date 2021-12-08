#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "boost/thread.hpp"
#include "Eigen/Dense"

#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Bool.h"
#include "progetto_esame/planner_msg.h"
#include "progetto_esame/odom_msg.h"
#include "progetto_esame/bag_msg1.h"

#define PI_2 1.570796327
#define PI 3.14159

class DiffDriveCtrl{

    public:
        DiffDriveCtrl(double x0, double y0, double yaw0);
        void run();     
        void Ctrl_loop();
		void Calcola_parametri_robot();
		void CalcolaDyn(); 
		void NonLinearControl(double &v_calcolata,double &w_calcolata,double x_rif,double y_rif,double yaw_rif,Eigen::Vector2d v_planner,double w_planner);
        void joint_states_cb( sensor_msgs::JointState);
        void odom_cb( progetto_esame::odom_msg);
        void planner_cb( progetto_esame::planner_msg);



    private:
		//------ node comunication 
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _planner_sub;
		bool _first_value_planner;
		ros::Publisher _joint_vel_pub[2];
		ros::Publisher _bag_pub;

		//------ variabili derivanti dal planner
		Eigen::Vector2d _acc_planner;
		double _theta_acc_planner;
		bool _stay_still;

		//------ variabili temporali
        ros::Time _current_time, _last_time;

		//------ parametri controllo 
		double _WheelMaxAngularVel=2;//2.5; //rad/s
		double _v_max;
		double _w_max;

		//------ variabili DiffDrive
		Eigen::Matrix<double,1,3> _q; //x y yaw 
		Eigen::Matrix<double,1,3> _dq; //vx vy w
		
		//------ parametri DiffDrive
		double _distanza_ruote;//distanza assiale ruote
		double _wheelRadius; //raggio ruote
		double _wheelOffsetY; 

		//------ Rosbag 
		progetto_esame::bag_msg1 _bagmsg;

		
};

DiffDriveCtrl ::DiffDriveCtrl(double x0, double y0, double yaw0) {

	_wheelOffsetY=0.12;
	_wheelRadius=0.033;
	_distanza_ruote =(_wheelOffsetY)*2; //distanza assiale ruote

	_q[0]=x0; //x
	_q[1]=y0; //y
	_q[2]=yaw0; //yaw

	_first_value_planner=false;


	_v_max=	_WheelMaxAngularVel*_wheelRadius;
	_w_max= 1.8*_WheelMaxAngularVel*_wheelRadius/_distanza_ruote;

	_odom_sub = _nh.subscribe("/my_robot/odom", 0, &DiffDriveCtrl::odom_cb, this);
	_planner_sub = _nh.subscribe("/my_robot/cmd_planner", 0, &DiffDriveCtrl::planner_cb, this);

	_joint_vel_pub[0] = _nh.advertise< std_msgs::Float64 > ("/my_robot/joint_left_controller/command", 0);
	_joint_vel_pub[1] = _nh.advertise< std_msgs::Float64 > ("/my_robot/joint_right_controller/command", 0);
	_bag_pub = _nh.advertise< progetto_esame::bag_msg1 > ("/my_robot/bag/control", 0);

}


void DiffDriveCtrl::odom_cb( progetto_esame::odom_msg  odom_msg) {

	_q[0] = odom_msg.position.x;
	_q[1] = odom_msg.position.y;
	_q[2] = odom_msg.yaw;
	_dq[0] = odom_msg.vel_x;
	_dq[1] = odom_msg.vel_y;
	_dq[2] = odom_msg.w;

}

void DiffDriveCtrl::planner_cb( progetto_esame::planner_msg  planner_msg) {
	_acc_planner<<planner_msg.acc_x,planner_msg.acc_y;
	_theta_acc_planner=planner_msg.theta_acc;
	_stay_still=planner_msg.stay_still;
	_first_value_planner=true;
}

void DiffDriveCtrl::NonLinearControl(double &v_calcolata,double &w_calcolata,double x_rif,double y_rif,double yaw_rif,Eigen::Vector2d v_planner,double w_planner){
	Eigen::Vector3d errore;
	Eigen::Vector3d errore_pos;
	Eigen::Matrix<double,3,3> Rot;
	Eigen::Vector2d virtual_input; //u1 u2

	//---- guadagni controllo 
	double K1; 
	double K2=10;
	double K3; 
	float csi=0.7;

	Rot<<	cos(_q[2]),		sin(_q[2]),		0,
			-sin(_q[2]),	cos(_q[2]),		0,
			0,				0,				1;
	
	errore_pos<< 	x_rif - _q[0],	y_rif - _q[1],	yaw_rif - _q[2];
	errore = Rot*errore_pos;
	
	K1=csi*sqrt(pow(w_planner,2)+K2*pow(v_planner.norm(),2)); 
	K3=csi*sqrt(pow(w_planner,2)+K2*pow(v_planner.norm(),2));

	virtual_input[0] = -K1*errore[0]; 
	virtual_input[1] = -K2*v_planner.norm()*sin(errore[2])*errore[1]/errore[2] -K3*errore[2];

	v_calcolata = v_planner.norm() * cos(errore[2]) - virtual_input[0]; 
	w_calcolata = w_planner - virtual_input[1];

	
	// ---Rosbag
	_bagmsg.err_x=errore_pos[0];
	_bagmsg.err_y=errore_pos[1];
	_bagmsg.err_z=errore_pos[2];
	_bagmsg.vel_x_ctrl=_dq[0];
	_bagmsg.vel_y_ctrl=_dq[1];
	_bagmsg.w_ctrl=_dq[2];
	_bagmsg.yaw_control=_q[2];

	
}


void DiffDriveCtrl::Ctrl_loop() {

	// --------variabili necessarie al calcolo del riferimento-----

	double dt;
	double theta_acc; 
	double x_dot_dot_planner; 
	double y_dot_dot_planner; 
	Eigen::Vector2d v_planner=Eigen::Vector2d::Zero();
	double w_planner;
	double theta_vel; 
	double x_dot_planner; 
	double y_dot_planner; 
	double last_y_dot_planner; 
	double x_rif=_q[0]; 
	double y_rif=_q[1]; 
	double yaw_rif=_q[2]; 

	// --------variabili necessarie al calcolo del controllo 

	double v_calcolata;
	double w_calcolata;
	std_msgs::Float64 cmd[2];
	
    ros::Rate r(10);
	
	while( !_first_value_planner ) usleep(0.1);

	_last_time =ros::Time::now();
    while(ros::ok()){ 
		//calcola riferimenti
		_current_time = ros::Time::now();
		dt = (_current_time - _last_time).toSec();
		
		if (!_stay_still ){

			v_planner[0] = _dq[0] + _acc_planner[0]*dt;
			v_planner[1] = _dq[1] + _acc_planner[1]*dt;
			if(v_planner.norm()>_v_max)v_planner = v_planner* _v_max/v_planner.norm();	
			
			theta_vel = atan2(v_planner[1],v_planner[0]);  

			x_dot_dot_planner = _acc_planner.norm()*cos(_theta_acc_planner);
			y_dot_dot_planner = _acc_planner.norm()*sin(_theta_acc_planner);
			
			x_dot_planner = v_planner.norm()*cos(theta_vel);
			y_dot_planner = v_planner.norm()*sin(theta_vel);

			if(x_dot_planner==0&&y_dot_planner==0)w_planner=0;
			else w_planner= (y_dot_dot_planner*x_dot_planner -x_dot_dot_planner*y_dot_planner)/(pow(x_dot_planner,2)+pow(y_dot_planner,2));
			

			if(fabs(w_planner)>_w_max)w_planner = w_planner* _w_max/fabs(w_planner); 

			x_rif =  _q[0] +x_dot_planner*dt;
			y_rif =  _q[1] + y_dot_planner*dt;
			yaw_rif= _q[2]+w_planner*dt;

			NonLinearControl(v_calcolata,w_calcolata, x_rif, y_rif, yaw_rif, v_planner, w_planner);
			
		}else if(_stay_still ){
			v_calcolata=0;
			w_calcolata=0;
		}
		
		_bagmsg.vel_lin_calcolata=v_calcolata;
		_bagmsg.vel_ang_calcolata=w_calcolata;

		cmd[0].data = ((2*v_calcolata - _distanza_ruote*w_calcolata)/(2*_wheelRadius));
		cmd[1].data = ((2*v_calcolata + _distanza_ruote*w_calcolata)/(2*_wheelRadius));
		
		_joint_vel_pub[0].publish(cmd[0]); 
		_joint_vel_pub[1].publish(cmd[1]);
		_bag_pub.publish(_bagmsg);

		//------------Debugging
		// std::cout<<"<<<<<<<<<< CTRL >>>>>>>>>>>>>>>>"<<std::endl;
        // std::cout<<"dt: "<<dt<<std::endl;
        // std::cout<<"_acc_planner: "<<_acc_planner<<std::endl;
        // std::cout<<"_theta_acc_planner: "<<_theta_acc_planner<<std::endl;
        // std::cout<<"x_dot_dot_planner: "<<x_dot_dot_planner<<std::endl;
        // std::cout<<"y_dot_dot_planner: "<<y_dot_dot_planner<<std::endl;
        // std::cout<<"theta_vel: "<<theta_vel<<std::endl;
        // std::cout<<"x_dot_planner: "<<x_dot_planner<<std::endl;
        // std::cout<<"y_dot_planner: "<<y_dot_planner<<std::endl<<std::endl;
        // std::cout<<"v_planner: "<<v_planner<<std::endl;
        // std::cout<<"w_planner: "<<w_planner<<std::endl<<std::endl;

        // std::cout<<"_v_max: "<<_v_max<<std::endl;
        // std::cout<<"_w_max: "<<_w_max<<std::endl;
        // std::cout<<"count: "<<count<<std::endl<<std::endl;

        // std::cout<<"x_rif: "<<x_rif<<std::endl;
        // std::cout<<"y_rif: "<<y_rif<<std::endl;
        // std::cout<<"k_giri: "<<k_giri<<std::endl;
        // std::cout<<"yaw_rif: "<<yaw_rif<<std::endl<<std::endl;

        // std::cout<<"Odom _x: "<<_q[0]<<std::endl;
        // std::cout<<"Odom _y: "<<_q[1]<<std::endl;
        // std::cout<<"Odom _yaw: "<<_q[2]<<std::endl<<std::endl;

        // std::cout<<"errore: "<<errore<<std::endl;
        // std::cout<<"K1: "<<K1<<std::endl;
        // std::cout<<"K3: "<<K3<<std::endl;
        // std::cout<<"virtual_input: "<<virtual_input<<std::endl<<std::endl;

        // std::cout<<"v_calcolata: "<<v_calcolata<<std::endl;
        // std::cout<<"w_calcolata: "<<w_calcolata<<std::endl;
        // std::cout<<"cmd[0].data: "<<cmd[0].data<<std::endl;
        // std::cout<<"cmd[1].data: "<<cmd[1].data<<std::endl;
        // std::cout<<"---------------------------"<<std::endl<<std::endl<<std::endl;



		//------------------------
		_last_time =_current_time;
        r.sleep();
    }
}


void DiffDriveCtrl::run() {
	boost::thread Ctrl_loop_t ( &DiffDriveCtrl::Ctrl_loop, this);
	ros::spin();	
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "robot_ctrl_node");

	if(argc>2){ 
        DiffDriveCtrl nd(atof(argv[1]),atof(argv[2]),atof(argv[3]));
		nd.run();
    }else{
        DiffDriveCtrl nd(5.0,1.0,1.570796327);
		nd.run();
    }


	return 0;
}
