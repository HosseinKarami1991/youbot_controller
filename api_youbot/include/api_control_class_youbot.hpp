/*
 *
 hri_control_Class.hpp
 *
 *  Created on: Jun 20, 2016
 *      Author: Kourosh Darvish
 */

#include <iostream>
#include <string>
#include <stdio.h>
#include <iterator>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Accel.h>
#include <std_msgs/Float64MultiArray.h>
#include "sensor_msgs/JointState.h"
#include "youbotcmd_msgs/ControlYoubot.h"
#include <vector>
#include <cmath>
#include <control_youbot/roc.h>
#include "youbotcmd_msgs/ControlYoubotGoalReachAck.h"
#include <cassert>
#include <numeric>
#include <limits>
#include <cmat/cmat.h>
#include <ortos/ortos.h>
#include<mocap_msgs/mocapvector.h>

#include <mutex>

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST


//# define M_PI		3.14159265358979323846	/* pi */

using namespace std;

class api_control_class_youbot {
	public:
        int objindex_=2;
		int counter1=0;
	 	int Arm_states=3;
	 	const int num_joints=5;
	 	bool hri_control_cmnd_flag[3];// check if necessary or not?
		bool control_ack_flag[3];
		bool cmnd_recived_flag;
		youbotcmd_msgs::ControlYoubot youbot_control_msg;
		vector<youbotcmd_msgs::ControlYoubot> youbot_control_msgs_vector;
		youbotcmd_msgs::ControlYoubotGoalReachAck controlAck;
		CTRL::ControllerInterface CtrlInterface;
        std::vector<double> mocappos_;
	//	 errorThresh_hri=0.01;

		 double qErrThresh_hri, angErrThresh_hri, linErrThresh_hri;
		 int errThreshDuration=5;

		api_control_class_youbot();
		void Callback_control(const youbotcmd_msgs::ControlYoubot& msg1);
		void Callback_mocap(const mocap_msgs::mocapvector::ConstPtr& mocvec);
		void publish_control_cmnd(int); //DEL
		void publish_control_ack(int armSate, string ack_type);
		void publish_robotArm_Q(void); // we need this for robot simulation in robot interface node.
		bool control_goal_error_check( string cmd,const int arm);
		void cmdValues_control( string cmd,const int arm, double *cmdValues);
		~api_control_class_youbot();

	private:
		ros::NodeHandle nh;
		ros::Subscriber sub_hri_control_cmnd;
		ros::Subscriber sub_JntSt;
		ros::Subscriber sub_ctrlTaskMsg;
        ros::Subscriber sub_mocap;
		ros::Publisher pub_robot_control_ack;
		ros::Publisher pub_ArmLeftQ;

        

		CMAT::TransfMatrix l_wTt, r_wTt, l_wTg, r_wTg;
		ortosdata::DataHelper odh;
		ortosdata::Vector6Container l_wTt_v6Ctr, r_wTt_v6Ctr;
		CMAT::Vect6 l_wTt_v6, r_wTt_v6;
		CMAT::Vect6 L_psi_, R_psi_;
		bool flag[3];
		int timeCounter[3];
		CMAT::Matrix q_, q_bar_;
		ortosdata::ArmFeedbackContainer l_fbk, r_fbk, _fbk_;
		double  **cartValsArray;
		double **jointValsArr;
		double toolValsArray[6];
};



