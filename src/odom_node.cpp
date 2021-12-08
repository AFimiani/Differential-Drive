#include <ros/ros.h>
#include <iostream>

#include <tf/transform_broadcaster.h>

#include <gazebo_msgs/ModelStates.h>

#include "sensor_msgs/JointState.h"
#include "Eigen/Dense"
#include "boost/thread.hpp"
#include "progetto_esame/odom_msg.h"
#include "progetto_esame/bag_msg2.h"

class OdomNode{

    public:
        OdomNode(double x0, double y0, double yaw0);
        void run();
        void Compute_Odometry();
        void joint_states_cb( sensor_msgs::JointState);
        void modelCallback4bag(const gazebo_msgs::ModelStates & msg);

    private:
        //comunicazione nodi
        ros::NodeHandle _nh;
        ros::Publisher _odom_pub;
        ros::Publisher _bag_pub;
        ros::Subscriber _js_sub;
        ros::Subscriber _model_state_sub;

        tf::TransformBroadcaster odom_broadcaster; 

        //variabili modello 
        double _wheelRadius;
        double _wheelDistance;
        //variabili spaziali 
        double _x;
        double _y;
        double _yaw;
        double _vx;
        double _vy;
        double _w;

        //variabili  
        Eigen::Vector2d _q;//phy_L phy_R
        Eigen::Vector2d _q_dot;//phy_L_dot phy_R_dot
        Eigen::Vector2d _q0;
        bool _first_js;
        double gazebo_odom[6];
        //varaibili temporali
        ros::Time current_time, last_time;

};



OdomNode::OdomNode(double x0, double y0, double yaw0){
    _odom_pub= _nh.advertise<progetto_esame::odom_msg>("/my_robot/odom",0);
    _bag_pub= _nh.advertise<progetto_esame::bag_msg2>("/my_robot/bag/odom",0);

    _js_sub = _nh.subscribe("/my_robot/joint_states", 0, &OdomNode::joint_states_cb, this);
    _model_state_sub = _nh.subscribe("/gazebo/model_states", 1, &OdomNode::modelCallback4bag, this);

    _wheelRadius=0.033; 
    
	double wheelOffsetY=0.12;
    _wheelDistance=(wheelOffsetY)*2;

    _x=x0;
    _y=y0;
    _yaw=yaw0;
    _vx=0;
    _vy=0;
    _w=0;
    _first_js=false;

    ROS_INFO("Odom node correctly initialized");
}

void OdomNode::joint_states_cb( sensor_msgs::JointState js ) {

	for(int i=0; i<2; i++ ) { 
		_q[i] = js.position[i]; //phy_L phy_R
		_q_dot[i] = js.velocity[i]; //phy_L_dot phy_R_dot
		if( !_first_js ) _q0[i] = js.position[i];//phy_L0 phy_R0
	}
    _first_js=true;
    
}

// per verificare l'andamento dell'errore sulla stima della posa, non usata nel controllo ,solo nei rosbag
void OdomNode::modelCallback4bag(const gazebo_msgs::ModelStates & msg){
    std::string model = "my_robot";
    bool found = false;
    int index = 0;
    double roll,pitch,yaw;
    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == model )
            found = true;
        else index++;
    }

    tf::Quaternion q( msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z, msg.pose[index].orientation.w);
    q.normalize();
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    gazebo_odom[0] = msg.pose[index].position.x;
    gazebo_odom[1] = msg.pose[index].position.y;
    gazebo_odom[2] = yaw;

    gazebo_odom[3] =msg.twist[index].linear.x;
    gazebo_odom[4] =msg.twist[index].linear.y;
    gazebo_odom[5] =msg.twist[index].angular.z;

}

void OdomNode::Compute_Odometry(){

    ros::Rate rate(1000); 
    double dt;
    double vk;
    double wk;
    Eigen::Vector2d delta_q;
    Eigen::Vector2d last_q;
    progetto_esame::odom_msg odom_msg;
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans_msg;
    while( !_first_js ) usleep(0.1);
    last_q=_q0;
    last_time=ros::Time::now();

    while(ros::ok()){ 
        current_time = ros::Time::now();
        
        //compute odometry - Runge-Kutta approximation
        dt = (current_time - last_time).toSec();

        while (dt==0){  //questo serve ad evitare la divisione per 0 se il programma va troppo veloce
            current_time = ros::Time::now();
            dt = (current_time - last_time).toSec();
        } 
        
        delta_q=_q-last_q;
        last_q=_q;

        vk=_wheelRadius *(delta_q[1] + delta_q[0])/(2*dt);
        wk=_wheelRadius *(delta_q[1] - delta_q[0])/(_wheelDistance*dt); //runge kutta 2 order
        if (wk==0)
        {
            _x=_x+vk*dt*cos(_yaw+wk*dt/2);
            _y=_y+vk*dt*sin(_yaw+wk*dt/2);

            _yaw=_yaw+wk*dt;
        }else{
            
            _x=_x+(vk/wk)*(sin(_yaw+wk*dt)-sin(_yaw)); // exact integration
            _y=_y-(vk/wk)*(cos(_yaw+wk*dt)-cos(_yaw));
            _yaw+=_wheelRadius *(delta_q[1] - delta_q[0])/(_wheelDistance);
        }
        _vx=vk*cos(_yaw);
        _vy=vk*sin(_yaw);
        _w=wk;

        odom_quat = tf::createQuaternionMsgFromYaw(_yaw);

        //transform broadcasting
        
        odom_trans_msg.header.stamp=current_time;
        odom_trans_msg.header.frame_id="odom";
        odom_trans_msg.child_frame_id="base_footprint";

        odom_trans_msg.transform.translation.x = _x;
        odom_trans_msg.transform.translation.y = _y;
        odom_trans_msg.transform.translation.z = 0.1;
        odom_trans_msg.transform.rotation = odom_quat;
        
        odom_broadcaster.sendTransform(odom_trans_msg);
       
        //odom publ
        
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.position.x = _x;
        odom_msg.position.y = _y;
        odom_msg.position.z = 0.1;
        odom_msg.yaw = _yaw;
        odom_msg.vel_x = _vx;
        odom_msg.vel_y = _vy;
        odom_msg.w = _w;

        _odom_pub.publish(odom_msg);

        // rosbag
        progetto_esame::bag_msg2 bagmsg;

        bagmsg.x = _x;
        bagmsg.y = _y;
        bagmsg.yaw = _yaw;
        bagmsg.vel_x = _vx;
        bagmsg.vel_y =_vy;
        bagmsg.w = _w;

        bagmsg.x_gaz = gazebo_odom[0];
        bagmsg.y_gaz =  gazebo_odom[1];
        bagmsg.yaw_gaz =  gazebo_odom[2];
        bagmsg.vel_x_gaz =  gazebo_odom[3];
        bagmsg.vel_y_gaz =  gazebo_odom[4];
        bagmsg.w_gaz =  gazebo_odom[5];

        _bag_pub.publish(bagmsg);

        last_time=current_time;

        
        rate.sleep();
        
    }

}
void OdomNode::run() {
	boost::thread Compute_Odometry_t( &OdomNode::Compute_Odometry, this );
	ros::spin();
}
int main(int argc, char** argv) {

	ros::init(argc, argv, "odom_node");
    if(argc>2){
        OdomNode nd(atof(argv[1]),atof(argv[2]),atof(argv[3]));
        nd.run();
    }else{
        OdomNode nd(2.0,2.0,1.570796327);
        nd.run();
    }
  


	return 0;
}
