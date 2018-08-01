#include "filter_and_control_tar.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include "geometry_msgs/PoseStamped.h"
#include <cmath>




namespace kitaf_navigation_ros_tool {

FilterAndControlTar::FilterAndControlTar(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, tfListener_{tfBuffer_} ,
		 dt(0.0),
		 X_state_akt_m(0,0,0,0),X_state_previous_m(0,0,0,0),X_t(0,0,0,0),X_t_vor(0,0,0,0),
		 y_t(0,0),y_t_pre(0,0),object_position_km(0,0,0),G(0,0),W_previous(0,0)
		 {


    //Initialization
    interface_.fromParamServer();

    //inint for all teime step
		H<<1,0,0,0,
		   0,0,1,0;

		I<<1,0,0,0,
		   0,1,0,0,
		   0,0,1,0,
		   0,0,0,1;
		//Eeinstellbare Parameter:
		R<<interface_.R_value, 0, //init Messrauschkovarianzmatrix
		   0,interface_.R_value;

		Q_1<<interface_.Q_shift,0,0,0,
			0,interface_.Q_velocity,0,0,
			0,0,interface_.Q_shift,0,
			0,0,0,interface_.Q_velocity;

    //init just for t=1, 1. step;
    n=1;
    m=0;
    P_0 << 1000, 0,0,0,
    		 0,1000,0,0,
			 0,0,1000,0,
			 0,0,0,1000;
	P_previous=P_0;

    A<<1,0.01,0,0,
        0,1,0,0,
 	   0,0,1,0.01,
 	   0,0,0,1;

    K_t<<0,0,
    	 0,0,
		 0,0,
		 0,0;


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/FilterAndControlTar.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&FilterAndControlTar::reconfigureRequest, this, _1, _2));

    //interface_.dummy_subscriber->registerCallback(&FilterAndControlTar::callbackSubscriber, this);
    //pub_sub.add_subscriber("dummy_subscriber", message_type="std_msgs::Header", description="example subscriber", default_topic="in_topic", no_delay=True, configurable=True)


//    interface_.path_subscriber->registerCallback(&FilterAndControlTar::pathcallbackSubscriber, this);

    pose_subscriber = nhPrivate.subscribe("/target_pose",
    		                                   interface_.queue_size_msg,
											   &FilterAndControlTar::posecallbackSubscriber,
                                               this,
                                               ros::TransportHints().tcpNoDelay());

    km_pose_publisher=nhPrivate.advertise<geometry_msgs::PoseStamped>("/km_pose",interface_.queue_size_msg);


    loop_timer_km_=nhPrivate.createTimer(ros::Rate(interface_.loop_timer_rate), &FilterAndControlTar::loopCallBack,this);

    rosinterface_handler::showNodeInfo();
}

void FilterAndControlTar::loopCallBack(const ros::TimerEvent& timer_event){

	//dt=1/interface_.loop_timer_rate;
	dt=timer_event.current_real.toSec() - timer_event.last_real.toSec();

	m=m+1;

	ROS_DEBUG_STREAM("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ new loop ++++++++++++++++++++++++++"<<std::endl
					<<"-------------n: "<<std::endl<<n<<std::endl
					<<"-------------dt: "<<std::endl<<dt<<std::endl
					<<"-------------timer_event.last_real.toSec(): "<<std::endl<<timer_event.last_real.toSec()<<std::endl

	);

	if(timer_event.last_real.toSec()<0.001 ){
		ROS_DEBUG_STREAM("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ dt too short ++++++++++++++++++++++++++"<<std::endl
				<<"-------------n: "<<std::endl<<n<<std::endl);

		//return;
	}else{

		if(pose_object.x()>0.001 ||pose_object.y()>0.001){

		ROS_DEBUG_STREAM("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ KM loop begin ++++++++++++++++++++++++++"<<std::endl
				<<"-------------n: "<<std::endl<<n<<std::endl
				<<"-------------m: "<<m<<std::endl
		        <<"-------------dt: "<<std::endl<<dt<<std::endl);


	//KM Begin:

		ROS_DEBUG_STREAM("*********************Vor new Step***********"<<std::endl
				<<"-------------X_state_akt_m: "<<std::endl<<X_state_akt_m<<std::endl
				<<"-------------X_state_previous_m: "<<std::endl<<X_state_previous_m<<std::endl
				<<"-------------P_previous: "<<std::endl<<P_previous<<std::endl
				<<"-------------P_akt_m: "<<std::endl<<P_akt_m<<std::endl
				<<"-------------K_t: "<<std::endl<<X_t<<std::endl
				<<"-------------X_t: "<<std::endl<<X_t<<std::endl
				<<"-------------P_akt: "<<std::endl<<P_akt<<std::endl
		);

	ROS_DEBUG_STREAM("*********************new step beginn*****************************"<<std::endl);
	//init X_state_previous_m

	//Aktuelle Messung y_t:
	y_t(0)=pose_object.x();
	y_t(1)=pose_object.y();

	//defind 4x4 Mtrix
	A(0,1)=dt;
	A(2,3)=dt;

	ROS_DEBUG_STREAM("*********************Parameter Matrix***********"<<std::endl
			<<"-------------Aktuelle Messung y_t: "<<std::endl<<y_t<<std::endl
			<<"-------------A: "<<std::endl<<A<<std::endl

	);


	//________________________________________________________________________________________________________________Prädiktion step
	X_state_akt_m=A*X_state_previous_m;
	P_akt_m=A*P_previous*A.transpose()+Q_1;//redefind kovarianzMatrix P

	ROS_DEBUG_STREAM("*********************Pradiktion***********"<<std::endl
			<<"-------------X_state_akt_m: "<<std::endl<<X_state_akt_m<<std::endl
			<<"-------------P_akt_m: "<<std::endl<<P_akt_m<<std::endl
	);

	//_______________________________________________________________________________________________________________Innovation step

	K_t=P_akt_m*H.transpose()*(H*P_akt_m*H.transpose()+R).inverse();//E*R*E.transpose()).transpose();
	X_t=X_state_akt_m+K_t*(y_t-H*X_state_akt_m); //aktuelle Schätzung
	P_akt=(I-K_t*H)*P_akt_m;

	ROS_DEBUG_STREAM("********************Innovation**********************"<<std::endl
			<<"-------------K_t: "<<std::endl<<K_t<<std::endl
			<<"-------------X_t: "<<"(n= "<<n<<" )"<<std::endl<<X_t<<std::endl
			<<"-------------P_akt: "<<std::endl<<P_akt<<std::endl
	);

	//new value in next step einsetzen:
	P_previous=P_akt;
	X_state_previous_m=X_t;
	n=n+1;


	//nach KM gefilterte objeckt positon
	object_position_km(0)=X_t(0); //x-value nach KM
	object_position_km(1)=X_t(2);//y-value nach KM

	ROS_DEBUG_STREAM("-------------out von KM: "<<std::endl<<object_position_km<<std::endl);

	//Publish:
	km_position_forPub.header.frame_id=interface_.map_frame_id;
	km_position_forPub.header.stamp=ros::Time::now();
	km_position_forPub.pose.position.x=X_t(0);
	km_position_forPub.pose.position.y=X_t(2);
	km_position_forPub.pose.position.z=0.0;

	km_pose_publisher.publish(km_position_forPub);



	}//if(m>5)
	}//if (dt too short)



}
double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double vz = boost::math::sign(a.cross(b).z());
    return vz * std::acos(a.normalized().dot(b.normalized()));
}



void FilterAndControlTar::posecallbackSubscriber(const geometry_msgs::PoseStamped::ConstPtr& msg_object_pose){
	msg_object_pose->pose.position;//.x;

	//pose_object = Eigen::Vector3d(msg_object_pose->pose.position.x,msg_object_pose->pose.position.y,msg_object_pose->pose.position.z);
	//poseReceiv=true;

	pose_object(0)=msg_object_pose->pose.position.x;
	pose_object(1)=msg_object_pose->pose.position.y;
	pose_object(2)=msg_object_pose->pose.position.z;

	km_position_forPub.pose.orientation = msg_object_pose->pose.orientation;


	ROS_DEBUG_STREAM("-------------pose from detector:  "<<std::endl<<msg_object_pose->pose.position<<std::endl);
}
/*
void FilterAndControlTar::callbackSubscriber(const Msg::ConstPtr& msg1) {

    // do your stuff here...
    Msg::Ptr newMsg = boost::make_shared<Msg>(*msg1);
    interface_.dummy_publisher.publish(newMsg);
}
*/

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void FilterAndControlTar::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace kitaf_navigation_ros_tool
