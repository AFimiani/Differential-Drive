#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <tf/tf.h>
#include "boost/thread.hpp"
#include "sensor_msgs/LaserScan.h" 
#include <ros/package.h>

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "progetto_esame/planner_msg.h"
#include "progetto_esame/odom_msg.h"
#include "progetto_esame/bag_msg.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
/*
    HP: tutti gli id dei qr code inziano con la sigla "id"
*/
#define PI_2 1.570796327
#define PI 3.14159

class ArtificialPotentialPlanner{
    public:
        ArtificialPotentialPlanner(double x0, double y0, double yaw0);
        void run();
        void Planning();
        void Min_loc_planner();

        void get_pose_cb( progetto_esame::odom_msg );
        void laser_cb( sensor_msgs::LaserScan );
        void Nav_logicCb(std_msgs::String msg); 
        void MoveToExplore();
        void send_task_msg(std::string mess);
        void GetGoal(double &x, double &y );
        void SetGoal(double x, double y );
        bool CalcolaPotenzialiArtificiali(double xf, double yf);
        void SaveMap();
        void MapCb( nav_msgs::OccupancyGrid map_msg);
        bool IsMinLoc();
    private:
        //comunicazione nodi
        ros::NodeHandle _nh;
        ros::Publisher _cmd_pub;
        ros::Publisher _task_status_pub;
        ros::Publisher _bag_pub;
        ros::Subscriber _odom_sub;
        ros::Subscriber _lidar_sub;
        ros::Subscriber _qr_data_sub;
        ros::Subscriber _map_sub_;

        //variabili robot
        Eigen::Vector3d _q; // x y yaw

        //--- parametri del task  
        Eigen::Matrix<double,4,2> _rooms; 
        std::string _id_wanted;
        std::vector<std::string> _id_found; 

        //parametri dei potenziali artificiali
        float _obstacle_range_of_influence=1.4;
        double _ka=25;//guadagno forza attrattiva
        double _kr=20; //guadagno forza repulsiva
        double _soglia_errore=0.05;
        double _soglia_minimo=0.8;

        //variabili del planner
        Eigen::Vector2d _actractive_force_close;
        Eigen::Vector2d _actractive_force_far;
        Eigen::Vector2d _repulsive_force;
        Eigen::Vector2d _resulting_force;
        Eigen::Vector2d _errore;

        double _actractive_potential_close;
        double _actractive_potential_far;
        double _repulsive_potential;
        double _resulting_potential;

        double _qf_x;
        double _qf_y;

        //Variabili necessarie alla logica di esplorazione
        bool _finished_exploration;
        bool _stay_still;
        bool _need_to_turn;

        Eigen::Matrix<double,1,4> _cardinals; 
        nav_msgs::OccupancyGrid map;

        Eigen::Vector2d _old_pos;
        ros::Time _chronometer;
        bool _chrono_started;

        Eigen::VectorXd _last_laser;

};

ArtificialPotentialPlanner::ArtificialPotentialPlanner(double x0, double y0, double yaw0){

    _cmd_pub= _nh.advertise<progetto_esame::planner_msg>("/my_robot/cmd_planner",0);
    _task_status_pub= _nh.advertise<std_msgs::String>("/my_robot/task_status",0);
    _bag_pub= _nh.advertise<progetto_esame::bag_msg>("/my_robot/bag/planner",0);
    _odom_sub = _nh.subscribe("/my_robot/odom", 0, &ArtificialPotentialPlanner::get_pose_cb, this);
    _lidar_sub = _nh.subscribe("/my_robot/laser/scan", 1, &ArtificialPotentialPlanner::laser_cb, this);
    _qr_data_sub = _nh.subscribe("/opencv/qr_data", 1, &ArtificialPotentialPlanner::Nav_logicCb, this);
    _map_sub_ = _nh.subscribe("map", 1, &ArtificialPotentialPlanner::MapCb, this);
    
    _actractive_force_close= Eigen::Vector2d::Zero();
    _actractive_force_far= Eigen::Vector2d::Zero();
    _repulsive_force= Eigen::Vector2d::Zero();
    _resulting_force= Eigen::Vector2d::Zero();
    _errore= Eigen::Vector2d::Zero();

    _actractive_potential_close = 0;
    _actractive_potential_far = 0;
    _repulsive_potential = 0;
    _resulting_potential = 0;
                
    _stay_still=false;
    _need_to_turn=false;
    _finished_exploration=false;

    _q[0]=x0;
    _q[1]=y0;
    _q[2]=yaw0;


    _old_pos<< _q[0],_q[1];
    _chrono_started=false;
    
    
    //------- CASO 1 
    _rooms  <<  11, 6, // Room 3 >Room 2 >Room 1 >Start
                7,11,
                3,8,
                x0,y0;
  

    //------- CASO 2: 

    // _rooms  <<  3,8, // Room 1 >Room 2 >Room 3 >Start
    //             7,11,
    //             11, 6,
    //             x0,y0;

    _nh.getParam("id_wanted",_id_wanted);
    SetGoal(_rooms(0,0),_rooms(0,1)); //TODO

}
void ArtificialPotentialPlanner::MapCb(  nav_msgs::OccupancyGrid map_msg){
    map=map_msg;
}

void ArtificialPotentialPlanner::SaveMap(){

    int threshold_occupied = 65;
    int threshold_free = 25;
    std::string mapdatafile = ros::package::getPath("progetto_esame");
    mapdatafile += "/map.pgm";
   
 

    ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    if (!out){
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
    }
    std::cout<<"map.info.resolution"<<map.info.resolution;
    std::cout<<"map.info.width"<<map.info.width;
    std::cout<<"map"<<map.info.height;
    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
            map.info.resolution, map.info.width, map.info.height);
    for(unsigned int y = 0; y < map.info.height; y++) {
        for(unsigned int x = 0; x < map.info.width; x++) {
            unsigned int i = x + (map.info.height - y - 1) * map.info.width;
            if (map.data[i] >= 0 && map.data[i] <= threshold_free) { 
                fputc(254, out);
            } else if (map.data[i] >= threshold_occupied) { 
                fputc(000, out);
            } else { 
                fputc(205, out);
            }
        }
    }

    fclose(out);
    send_task_msg("Map Saved");
    ROS_INFO("Done\n");

}
void ArtificialPotentialPlanner::SetGoal(double x, double y ){
    _nh.setParam("xf",x);
    _nh.setParam("yf",y);
}
void ArtificialPotentialPlanner::GetGoal(double &x, double &y ){
    _nh.getParam("xf",x);
    _nh.getParam("yf",y);
}


bool ArtificialPotentialPlanner::CalcolaPotenzialiArtificiali(double xf, double yf){

    _errore<<xf-_q[0],yf-_q[1];
    _actractive_force_close=_ka*_errore;
    _actractive_force_far=_ka*(_errore/_errore.norm());
    _resulting_force=(_errore.norm()>=1)?(_actractive_force_far +_repulsive_force):(_actractive_force_close + _repulsive_force);

    _actractive_potential_close = _ka*pow(_errore.norm(),2);
    _actractive_potential_far = _ka*_errore.norm();
    _resulting_potential=(_errore.norm()>=1)?(_actractive_potential_far +_repulsive_potential):(_actractive_potential_close + _repulsive_potential);
    

    progetto_esame::bag_msg msg;
    msg.tot_for_x=_resulting_force[0];
    msg.tot_for_y=_resulting_force[1];
    msg.planner_x=_q[0];
    msg.planner_y=_q[1];
    _bag_pub.publish(msg);

    return _errore.norm()<_soglia_errore;
}

void ArtificialPotentialPlanner::Nav_logicCb(std_msgs::String msg){ 
    bool id_already_found=false;

    if (msg.data[0]=='i'&& msg.data[1]=='d') // Se individua un codice id
    {
        if(!_id_found.empty()){ //check se l'aveva già trovato
            for(int j=0; j<_id_found.size();j++){if(msg.data==_id_found[j])id_already_found=true; }
        }
        if(!id_already_found){ //se non l' ha trovato
            _id_found.push_back(msg.data); //aggiungilo alla lista 
            send_task_msg("Found qr data: "+ msg.data +" in room number "+std::to_string(_id_found.size()));
            if (msg.data==_id_wanted  || _id_found.size()>=_rooms.rows()) //se è quello desiderato o se sono finiti torna a Start
            {   
                if(msg.data==_id_wanted )send_task_msg(" Id Found in the explored enviroment");
                else send_task_msg("Id non found in the explored enviroment");
                send_task_msg(" Robot coming back to starting position");
                SetGoal(_rooms(3,0),_rooms(3,1));
                _finished_exploration=true;    
            }else{ //altrimenti procedi per la prossima stanza
                send_task_msg("Found qr data: "+msg.data+ " different from desired id: " + _id_wanted +", keep searching");
                SetGoal(_rooms(_id_found.size(),0),_rooms(_id_found.size(),1));
                send_task_msg("Destination set to ("+ std::to_string(_rooms(_id_found.size(),0)) +", "+ std::to_string(_rooms(_id_found.size(),1))+ ")");
            }
            _need_to_turn=false; 
        }
        
    }
    
}

void ArtificialPotentialPlanner::get_pose_cb( progetto_esame::odom_msg odom_msg ){
    
	_q[0]=odom_msg.position.x;
	_q[1]=odom_msg.position.y;
    _q[2]=odom_msg.yaw;

}


//quando riceve le informazioni dal topic del laser calcola le forze repulsive dovute agli ostacoli
void ArtificialPotentialPlanner::laser_cb( sensor_msgs::LaserScan laser){
    /*
        comportamento laser:
        il laser prende 360 campioni e ha un angolo di visione di 360° 
        ogni raggio è distanziato di un angolo pari a laser.angle_increment
    */
    bool FoundObstacle=false;
    float distance_from_obstacle=_obstacle_range_of_influence;
    double phy;
    int i_dist_min=0;
    int gamma = 2;

    Eigen::Vector2d obstacle_pos;
    Eigen::Vector2d robot_pos;
    Eigen::Vector2d rho_versore;
    Eigen::Vector2d phi_versore;
    Eigen::Vector2d df_rho;
    Eigen::Vector2d df_phy;
    Eigen::Vector2d temp;

    
    

    int obstacle_count=0;
    int j=laser.ranges.size()-1;

    temp=Eigen::Vector2d::Zero(); //si parte dall'ipotesi di non avere ostacoli 
    obstacle_pos=Eigen::Vector2d::Zero(); 
    rho_versore=Eigen::Vector2d::Zero(); 
    int nray4obstacle=0;
    for (int i = 0; i <=j; i++) //scorriamo i raggi del laser
    {
        if(laser.ranges[i]<= _obstacle_range_of_influence && i!=j) { //nuovo ostacolo
            FoundObstacle=true;
            if(i==0){ 
                while(laser.ranges[j]<= _obstacle_range_of_influence && nray4obstacle<30){ // caso in cui a 360 e 0 abbiano un ostacolo e data la non circolaritàò del vettore venga visto come due 
                    if(laser.ranges[j]<distance_from_obstacle){ //punto piu vicino all'ostacolo 
                        distance_from_obstacle=laser.ranges[j];
                        i_dist_min=j;
                    }
                    j--;
                    nray4obstacle++;
                }
            }
            
            if(laser.ranges[i]<distance_from_obstacle){ //punto piu vicino all'ostacolo 
                distance_from_obstacle=laser.ranges[i];
                i_dist_min=i;
            }
            nray4obstacle++;

        }else if(FoundObstacle ||nray4obstacle>30){ //  avevamo trovato l'ostacolo e adesso non lo vediamo più  + dividi ostacoli grandi
            FoundObstacle=false;
            nray4obstacle=0;
            phy= i_dist_min*laser.angle_increment +_q[2] ;// cooridnate polari rispetto al robot 
            obstacle_pos[0]=_q[0] +  distance_from_obstacle*cos(phy); 
            obstacle_pos[1]=_q[1] +  distance_from_obstacle*sin(phy); //rispetto al mondo 
            
            // con coordinate polari 
            double gradiente_eta;
            robot_pos<<_q[0],_q[1];
            rho_versore = (robot_pos-obstacle_pos)/((robot_pos-obstacle_pos).norm());
            phi_versore<< -rho_versore[1],rho_versore[0]; // perpendicolare 
            df_rho<< cos(phy), sin(phy);
            df_phy<< -distance_from_obstacle*sin(phy),distance_from_obstacle*cos(phy);//distance_from_obstacle=rho
            gradiente_eta=df_rho.dot(rho_versore)+1/distance_from_obstacle*df_phy.dot(phi_versore); //prodotto scalare
            temp += (_kr/pow(distance_from_obstacle,2))*pow(1/distance_from_obstacle- 1/_obstacle_range_of_influence, gamma-1)*fabs(gradiente_eta)*rho_versore;

            _repulsive_potential+= _kr/gamma*pow(1/distance_from_obstacle-1/_obstacle_range_of_influence,gamma); 
            distance_from_obstacle=_obstacle_range_of_influence; //reset variabile distanza
            obstacle_count++;
        }
    }
    if(obstacle_count!=0) _repulsive_force=temp;
    else _repulsive_force=Eigen::Vector2d::Zero();

    _cardinals[0]=laser.ranges[laser.ranges.size()/2]; //dietro
    _cardinals[1]=laser.ranges[0]; //avanti
    _cardinals[2]=laser.ranges[laser.ranges.size()-laser.ranges.size()/4]; //destra 
    _cardinals[3]=laser.ranges[laser.ranges.size()/4]; //sinistra

    //----Debugging 
    // std::cout<<"<<<<<<<<<< PLANNER >>>>>>>>>>>>>>>>"<<std::endl;
    // std::cout<<"obstacle_count: "<<obstacle_count<<std::endl;
    // if(_errore.norm()>=1)std::cout<<"_actractive_force: "<<_actractive_force_far<<std::endl;
    // else std::cout<<"_actractive_force: "<<_actractive_force_close<<std::endl;
    // std::cout<<"_repulsive_force: "<<_repulsive_force<<std::endl;
    // std::cout<<"_resulting_force: "<<_resulting_force<<std::endl;
    // if(_resulting_force.norm()<5) std::cout<<"_resulting_force norma: "<<_resulting_force.norm()<<std::endl;
    // std::cout<<"_repulsive_force: "<<_repulsive_force<<std::endl;
    // std::cout<<"_cardinals: "<<_cardinals<<std::endl;
    // std::cout<<"_qf_x: "<<_qf_x<<std::endl;
    // std::cout<<"_qf_y: "<<_qf_y<<std::endl;


}


void ArtificialPotentialPlanner::MoveToExplore(){
    
    _need_to_turn=false;
    if(_cardinals[1] >=1.0+ _obstacle_range_of_influence){ //se davanti a te non hai ostacoli vicini avanza di un metro 
        SetGoal(_q[0] +  cos(_q[2]) , _q[1] +  sin(_q[2]) ); // Pos = Pos_attuale+ [dist*cos(yaw); dist* sin(yaw)]  
        send_task_msg("Destination set to ("+ std::to_string(_q[0] +  cos(_q[2])) +", "+ std::to_string(_q[1] +  sin(_q[2]))+ ")");
    }else{
        bool ra = (rand() % 2 >=0.5);
        double psi;
        if(_cardinals[2+int(ra)] >=1.0+ _obstacle_range_of_influence){
            psi= (int(ra)==0)? -PI_2:PI_2; 
            SetGoal(_q[0] +  cos(psi+_q[2]) , _q[1] +  sin(psi+_q[2]) );
            send_task_msg("Destination set to ("+ std::to_string(_q[0] +  cos(psi+_q[2])) +", "+ std::to_string(_q[1] +  sin(psi+_q[2]))+ ")");

        }else if (_cardinals[2+int(!ra)] >=1.0+ _obstacle_range_of_influence)
        {
            psi= (int(!ra)==0)? -PI_2:PI_2; 
            SetGoal(_q[0] +  cos(psi+_q[2]) , _q[1] +  sin(psi+_q[2]) );
            send_task_msg("Destination set to ("+ std::to_string(_q[0] +  cos(psi+_q[2])) +", "+ std::to_string(_q[1] +  sin(psi+_q[2]))+ ")");
        }else{
            SetGoal(_q[0] -  cos(_q[2]) , _q[1] -  sin(_q[2]) );
            send_task_msg("Destination set to("+ std::to_string(_q[0] - cos(_q[2])) +", "+ std::to_string(_q[1] - sin(_q[2]))+ ")");

        }
    }
}

bool ArtificialPotentialPlanner::IsMinLoc(){
   
    bool th = _resulting_force.norm()<_soglia_minimo && _errore.norm()>=1 && _resulting_potential>0;
    bool stuck=false;
    Eigen::Vector2d current_pos;
    current_pos<<_q[0],_q[1];

    if( (_old_pos -current_pos).norm()<0.8 ){
        if(!_chrono_started){
            _chronometer=ros::Time::now();
            _chrono_started=true;
        }
    }else{
        _old_pos=current_pos;
        _chrono_started=false;
    }

    if(_chrono_started &&(ros::Time::now()-_chronometer ).toSec()>60){
        stuck=true;
        _chrono_started=false;
    }
    
    return th||stuck;
}

void ArtificialPotentialPlanner::Planning(){

    ros::Rate rate(10);
    progetto_esame::planner_msg msg;
    double yaw_f=0;
    send_task_msg("Start looking for "+_id_wanted);
    send_task_msg("Destination set to ("+ std::to_string(_qf_x) +", "+ std::to_string(_qf_y)+ ")");

    while(ros::ok()){
        GetGoal(_qf_x,_qf_y);

        if(CalcolaPotenzialiArtificiali(_qf_x,_qf_y)){  // se sei arrivato alla posizione finale 
            if(!_finished_exploration){
                send_task_msg("Starting exploration logic");
                //start turning
                yaw_f=_q[2]-2*PI;
                _need_to_turn=true;
                while(ros::ok() && _q[2]>=yaw_f && _need_to_turn){
                    msg.acc_x=  sin(_q[2]);
                    msg.acc_y= -cos(_q[2]);
                    msg.theta_acc= atan2(-cos(_q[2]),sin(_q[2]) );  
                    _cmd_pub.publish(msg);

                    rate.sleep();
                }
                //se _need_to_turn è ancora true significa che non ha trovato il qr code e che quindi bisogna continuare a girare 
                if(_need_to_turn)MoveToExplore();

            }else {
                if(!_stay_still){
                    send_task_msg("Saving the map");
                    SaveMap();//chiama solo una volta
                }
                _stay_still=true; //  non mettendo questo bool il controllo convergerebbe lo stesso al punto desiderato, ma preferisco "spegnere i motori"
            }
        }else if(IsMinLoc() ){//verifica problema di minimo locale 
            ROS_INFO("[WARNING!] Local Minimum reached");
            Min_loc_planner();
        }

        msg.acc_x=_resulting_force[0];
        msg.acc_y=_resulting_force[1];
        msg.theta_acc= atan2(_resulting_force[1],_resulting_force[0]);  
        msg.stay_still= _stay_still;
        _cmd_pub.publish(msg);
        

        //----Debugging 
        // std::cout<<"<<<<<<<<<< PLANNER >>>>>>>>>>>>>>>>"<<std::endl;
        // if(_resulting_force.norm()<1.0)std::cout<<"_resulting_force: "<<_resulting_force.norm()<<std::endl;
        // if(_errore.norm()>=1)std::cout<<"_actractive_force: "<<_actractive_force_far<<std::endl;
        // else std::cout<<"_actractive_force: "<<_actractive_force_close<<std::endl;
        // std::cout<<"_repulsive_force: "<<_repulsive_force<<std::endl;
        // std::cout<<"_resulting_force: "<<_resulting_force.norm()<<std::endl;
        // std::cout<<"_resulting_potential: "<<_resulting_potential<<std::endl;
        // std::cout<<"_errore.norm(): "<<_errore.norm()<<std::endl;
        // std::cout<<"_cardinals: "<<_cardinals<<std::endl;
        rate.sleep();
    }

}

void ArtificialPotentialPlanner::Min_loc_planner(){
    /* logica 
        una direzione avente come componente x una randomica variazione della posizione attuale nell'intervallo [-6,6]
        e come componente y la direzione in cui il robot non vede ostacoli 
        caso di ricaduta in min -> direzione scelta sempre in un cono scelto casualmente tra destra e sinistra
    */
    ros::Rate rate(100);
    progetto_esame::planner_msg msg;
    Eigen::Vector2d destinazione;
    double psi =0; 
    // double psi =_q[2]-PI+((rand()%3 -1)*PI/8);
    double distanza=6; // TODO DA TESTARE 
    bool done=false;
    send_task_msg("[WARNING!] Local Minimum reached, start repositioning ");
    
    int index_clear_path=0;
    for(int i=1; i<4;i++){
        if(_cardinals[i]>_cardinals[index_clear_path]){
            index_clear_path=i;
        }
    }
    switch(index_clear_path){
        case 0: 
            psi=_q[2]-PI; 
            break;
        case 1:
            psi=_q[2]; 
            break;
        case 2:
            psi=_q[2]-PI/4; 
            break;
        case 3:
            psi=_q[2]+PI/4; 
            break;
    }


    destinazione << _q[0]+ (rand()%13 -6 ),
                    _q[1]+ distanza* sin(psi );
    send_task_msg("Temporary destination set to ("+ std::to_string(destinazione[0]) +", "+std::to_string(destinazione[1])+ ")") ;

    ros::Time tstop = ros::Time::now(); 
    while(ros::ok() &&  !done && (ros::Time::now()-tstop).toSec()<60 ){

        
        if(CalcolaPotenzialiArtificiali(destinazione[0],destinazione[1]))done=true;
        else if(IsMinLoc()){ 
            send_task_msg("[WARNING!] Another Local Minimum reached, repositioning recalculation ");
            int ra = int(_cardinals[2]<_cardinals[3]); //ra 0 destra
            psi =_q[2]-PI/4 + ra*PI/2;
            // psi =_q[2]-PI/2 + ra*PI +((rand()%3 -1)*PI/8);
            destinazione << _q[0]+ distanza* cos(psi),
                            _q[1]+ distanza* sin(psi);
            send_task_msg("Temporary destination set to("+ std::to_string(destinazione[0]) +", "+std::to_string(destinazione[1])+ ")") ;
        }

        //----- pubblica soluzione 
        msg.acc_x=_resulting_force[0];
        msg.acc_y=_resulting_force[1];
        msg.theta_acc= atan2(_resulting_force[1],_resulting_force[0]);  
        msg.stay_still=_stay_still;
        _cmd_pub.publish(msg);

        rate.sleep();
    }
    send_task_msg("Exploration of the rooms resumed ");
}

void ArtificialPotentialPlanner::run() {
	boost::thread Planning_t( &ArtificialPotentialPlanner::Planning, this );
	ros::spin();
}

void ArtificialPotentialPlanner:: send_task_msg(std::string mess){
    std_msgs::String task_msg;
    task_msg.data=mess;
    _task_status_pub.publish(task_msg);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "artificialpotentialplanner_node");

    if(argc>2){
        ArtificialPotentialPlanner nd(atof(argv[1]),atof(argv[2]),atof(argv[3]));
        nd.run();
    }else{
        ArtificialPotentialPlanner nd(5.0,1.0,1.570796327);
        nd.run();
    }
    

	return 0;
}