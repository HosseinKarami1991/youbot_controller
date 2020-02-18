/*
 * interface3.cc
 *
 *  Created on: Dec 2, 2016
 *      Author: nasa
 */



//CPP
#include <api_control_class_youbot.hpp>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <iostream>
#include <vector>
#include <ortos/ortos.h>
#include <cmat/cmat.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <ros/ros.h>
#include <control_youbot/roc.h>
#include <time.h>
#include "youbotcmd_msgs/ControlYoubot.h"
#include "youbotcmd_msgs/ControlYoubotGoalReachAck.h"
#include <control_youbot/ortosdata_defines.h>
//#include "command_consoleAPI.h"
#include "control_youbot/roc.h"
#include "knowledge_msgs/knowledgeSRV.h"

using std::cout;
using std::endl;
using std::vector;

std::atomic_int atom_arm { 0 };
std::atomic_bool atom_checkResp { false };
std::mutex _lock;
ros::ServiceClient knowledgeBase_client;
using namespace ortosdata;

struct BothArmsReached
{
    bool l_GR, r_GR;
    CTRL::ControllerInterface* ci_ptr;

    BothArmsReached(CTRL::ControllerInterface* ciPtr) :
            l_GR(false), r_GR(false), ci_ptr(ciPtr)
    {
    }

    bool Reached()
    {
        if (ci_ptr->IsGoalReached((int) ArmNames::Left))
            l_GR = true;
        if (ci_ptr->IsGoalReached((int) ArmNames::Right))
            r_GR = true;
        if (l_GR && r_GR)
        {
            l_GR = r_GR = false;
            return true;
        }
        else
        {
            return false;
        }
    }
};

void goal_reached_thread()
{
	int ret;
	ortos::Task* task = ortos::Task::GetInstance();
	task->SetSampleTime(10 * ortos::constants::oneMillisecond);

	ret = task->CreateSync("Goal_Reached_thread_api_youbot");

	ortos::DebugConsole::Write(ortos::LogLevel::info, "thread", "Task created");
	if (ret != ORTOS_RV_OK) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "Error creating the task");
		task->Exit();
	}
	ortos::xcom::XCOMInterface* xcom = ortos::xcom::XCOMInterface::GetInstance();
	CTRL::ControllerInterface ctrlInterface;

	while (task->Continue()) {
		if (atom_checkResp) {
			if (ctrlInterface.IsGoalReached(atom_arm)) {
				atom_checkResp = false;
				_lock.lock();
//				std::cout << tc::yel << "Arm [" << ortosdata::arm2String.at(atom_arm) << "] Goal Reached!" << tc::none << std::endl;
				_lock.unlock();
			}
		}
		task->WaitPeriod();
	}
}

/**
 * Node main function
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "API_youbot_CONTROL");
	ros::NodeHandle nh;

	int ret;
	ortos::Task* task = ortos::Task::GetInstance();
	task->SetSampleTime(CTRL::RateDefines::TaskPeriodMillis * ortos::constants::oneMillisecond);
	ret = task->CreateSync("API_CONTROL_LOWER_PLANNER_youbot");
//	ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Task created");
	if (ret != ORTOS_RV_OK) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Error creating the task");
		task->Exit();
	}
	ortos::xcom::XCOMInterface* xcom = ortos::xcom::XCOMInterface::GetInstance();
	CTRL::ControllerInterface ctrlInterface_;
	ortosdata::DataHelper odh;
    BothArmsReached bar(&ctrlInterface_);

	api_control_class_youbot control_obj;// initializing control_hri class

	ortosdata::Vector6Container l_wTt_v6Ctr, r_wTt_v6Ctr;
	CMAT::Vect6 l_wTt_v6, r_wTt_v6;
	//CMAT::Vect6 l_wTg_v6, r_wTg_v6;
	//CMAT::Vect6 v6goal;

	CMAT::Vect6 L_psi_, R_psi_;
	CMAT::TransfMatrix l_wTt, r_wTt, l_wTg, r_wTg;

	bool taskCompleted = false, running = false;
	unsigned int iter = 0;
	int arm;

	ortosdata::BoolContainer l_cmd, r_cmd;
	l_cmd.d = false;
	r_cmd.d = false;
    
    ortosdata::Vector6Container reachinggoal;
    odh.AddDataGroup(reachinggoal, "mygoalreached");
    ortosdata::Vector6Container objectPoseCtr1;
    odh.AddDataGroup(objectPoseCtr1,ortosdata::topicnames::mocapRobotPose);
    std::fill(objectPoseCtr1.d.data, objectPoseCtr1.d.data + 6, 0.0);
    xcom->Publish(ortosdata::topicnames::mocapRobotPose, objectPoseCtr1);
    knowledgeBase_client=nh.serviceClient<knowledge_msgs::knowledgeSRV>("knowledgeService");


	/* hri commands
	ret = xcom->Subscribe(ortosdata::topicnames::goalReachedHRI+ "/left", l_cmd);
	if (ret != ORTOS_RV_OK) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Error while subscribing to the goalReachedHRI Left Arm");
	}
	ret = xcom->Subscribe(ortosdata::topicnames::goalReachedHRI+ "/right", r_cmd);
		if (ret != ORTOS_RV_OK) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Error while subscribing to the goalReachedHRI Right Arm");
		}
*/
	int BiArm1,BiArm2;
	bool holdingmodeCmnd, GripperCmnd;//GripperCmnd: 0=close, 1: open
	bool error_check=0;
	const int No_Gripper=1;
	const int Arm_state=1; //left,right, bimanual
	bool GripperFlag[No_Gripper];
	bool HoldModeFlag[No_Gripper];
	bool stopArmFlag[No_Gripper];

	ros::Time sysTime[No_Gripper];
	double timeDiff[No_Gripper];// seconds
	long double sysTimeMS[No_Gripper];
	ros::Time sysTimeGripper[No_Gripper];
	long double sysTimeMSGripper[No_Gripper] ;
	int ack_counter[Arm_state];
	unsigned long int counter=0;
	//unsigned int iter = 0;
	//int arm;

	for (int mm=0;mm<No_Gripper;mm++)
	{	GripperFlag[mm]=true;
		HoldModeFlag[mm]=true;
		stopArmFlag[mm]=true;
		sysTime[mm]=ros::Time::now();sysTimeMS[mm]=0; sysTimeGripper[mm]=ros::Time::now();sysTimeMSGripper[mm]=0 ;timeDiff[mm]=0.0;
	}
	for (int mm=0;mm<Arm_state;mm++)
		ack_counter[mm]=0;

youbotcmd_msgs::ControlYoubot input;
	string cmdRecvd;
	std::vector<double> cmdValues;
	double jointValsArr[ortosdata::numJoints], cartValsArray[6],wTo[6],wTg[6], jointValArray[7];
	CTRL::CtrlMode ctrlMode;

//	CommandConsole::ParsRet parsRetVal;
//	CommandConsole console;
	std::thread grt(goal_reached_thread);

	/// Facility sleep, just to get the console output printed after all the ortostask-related prints
//	std::this_thread::sleep_for(std::chrono::seconds(1));

//	console.PrintCommandList();

	 double qErrThresh_hri=0.001*10, angErrThresh_hri=4 * M_PI / 180.0, linErrThresh_hri=0.01*2;
	 bool flag=0;

	 ros::Rate loop_rate(100);

	while (ros::ok() &&  !taskCompleted && task->Continue()) {
		xcom->Synchronize();
//		cout<<counter<<endl;
           
		if (counter==0){
			 ctrlInterface_.StopArm(0);
			 ctrlInterface_.SetHoldingMode(0,0);
			 ctrlInterface_.OpenGripper(0);
			
			 double jointValArrayLeft[5]={ 0.6, -0.3, -0.067, 0.6, 0.472}; //left
			 ctrlInterface_.SetJointsPosition(jointValArrayLeft,0);
			 
//			 for(int j=0;j<5;j++)
//			 {
//				 ctrlInterface_.IncreaseGripHoldingForce(0);
//				 ctrlInterface_.IncreaseGripHoldingForce(1);
//			 }
		}
		

         
		counter++;
		for(int i=0;i<6;i++){
			objectPoseCtr1.d.data[i]=control_obj.mocappos_[i];
		}
		odh.WriteVector6(objectPoseCtr1, ortosdata::topicnames::mocapRobotPose);
      //  cout<<"psirobt "<<control_obj.mocappos_[0]<<" xrobot:  "<<control_obj.mocappos_[1]<<" yrobot: "<<control_obj.mocappos_[2]<<endl;

		if (control_obj.cmnd_recived_flag==false)
		{         cout<<"************** i passed if "<<endl;
	              
       
			
			for (int i=0;i<control_obj.youbot_control_msgs_vector.size();i++){

				input=control_obj.youbot_control_msgs_vector[i];

				if(input.Activation==0){
					cmdRecvd=input.youbotArm.youbotCmdType;
					arm=input.youbotArm.youbotIndex;
					if (cmdRecvd=="cartPos"){
						cartValsArray[0]=input.youbotArm.cartGoal.cartesianPosition[0];
						cartValsArray[1]=input.youbotArm.cartGoal.cartesianPosition[1];
						cartValsArray[2]=input.youbotArm.cartGoal.cartesianPosition[2];
						cartValsArray[3]=input.youbotArm.cartGoal.cartesianPosition[3];
						cartValsArray[4]=input.youbotArm.cartGoal.cartesianPosition[4];
						cartValsArray[5]=input.youbotArm.cartGoal.cartesianPosition[5];
						ctrlInterface_.SetCartesianPosition(cartValsArray, arm);

					}
					else if (cmdRecvd=="jointPos"){
						jointValArray[0]=input.youbotArm.cartGoal.youbotJointPose[0];
						jointValArray[1]=input.youbotArm.cartGoal.youbotJointPose[1];
						jointValArray[2]=input.youbotArm.cartGoal.youbotJointPose[2];
						jointValArray[3]=input.youbotArm.cartGoal.youbotJointPose[3];
						jointValArray[4]=input.youbotArm.cartGoal.youbotJointPose[4];
					
						ctrlInterface_.SetJointsPosition(jointValArray,arm);
					} else cout<<"ERROR:: control command arrived, Activation=0"<<endl;
					control_obj.control_ack_flag[arm]=false;

				
				}else if (input.Activation==2){
					arm=input.youbotArm.youbotIndex;
					GripperCmnd=input.youbotArm.value;
					if (GripperCmnd==false){
						cmdRecvd="closeGrip";
						ctrlInterface_.CloseGripper(arm);
					}
					if (GripperCmnd==true){
						cmdRecvd="openGrip";
						ctrlInterface_.OpenGripper(arm);
					}

					GripperFlag[arm]=false;
					sysTimeGripper[arm]=ros::Time::now();
//					sysTimeMSGripper[arm]=sysTimeGripper[arm]*1000;

				}else if (input.Activation==3){
					cmdRecvd="stopArm";
					arm=input.youbotArm.youbotIndex;
					ctrlInterface_.StopArm(arm);
					stopArmFlag[arm]=false;
					sysTimeGripper[arm]=ros::Time::now();
//					sysTimeMSGripper[arm]=sysTimeGripper[arm]*1000;

				}
				else if (input.Activation==4)
				{

					cmdRecvd="holdMode";
					arm=input.youbotArm.youbotIndex;
					holdingmodeCmnd=input.youbotArm.value;
					ctrlInterface_.SetHoldingMode(holdingmodeCmnd,arm);
					HoldModeFlag[arm]=false;
					sysTimeGripper[arm]=ros::Time::now();
//					sysTimeMSGripper[arm]=sysTimeGripper[arm]*1000;

				}
				      	else if (input.Activation==5)
				{
                   cmdRecvd="mobilecartpos";
                   arm=input.youbotArm.youbotIndex;
                        //cartesianPosition=[psi_youbot,x_youbot,y_youbot,psi_object,x_object,y_object]
				                     
				    knowledge_msgs::knowledgeSRV knowledge_msg;
					knowledge_msg.request.reqType="nextplace";
					knowledge_msg.request.Name="";
					knowledge_msg.request.requestInfo="Pose";

				   if(knowledgeBase_client.call(knowledge_msg)){
				    control_obj.objindex_ = (int)knowledge_msg.response.pose.back();

				   }
                   	cartValsArray[0]=input.youbotArm.cartGoal.cartesianPosition[0];
					cartValsArray[1]=input.youbotArm.cartGoal.cartesianPosition[1];
					cartValsArray[2]=input.youbotArm.cartGoal.cartesianPosition[2];
					cartValsArray[3]=input.youbotArm.cartGoal.cartesianPosition[3];
					cartValsArray[4]=input.youbotArm.cartGoal.cartesianPosition[4];
					cartValsArray[5]=input.youbotArm.cartGoal.cartesianPosition[5];
                   /*
					cartValsArray[0]=control_obj.mocappos_[0];
					cartValsArray[1]=control_obj.mocappos_[1];
					cartValsArray[2]=control_obj.mocappos_[2];
					cartValsArray[3]=control_obj.mocappos_[3];
					cartValsArray[4]=control_obj.mocappos_[4];
					cartValsArray[5]=control_obj.mocappos_[5];
                   */
					//cout<<"psirobot: "<<cartValsArray[0]<<"xrobot: "<<cartValsArray[1]<<" yrobot: "<<cartValsArray[2]<<endl;
                    //cout<<"psiobj: "<<cartValsArray[3]<<" xobj: "<<cartValsArray[4]<<" yobj: "<<cartValsArray[5]<<endl;
					ctrlInterface_.SetMobileCartesianPosition(cartValsArray,arm);

				}
		
				else
				{
					cmdRecvd="";
					cout<<"input.Activation: "<<input.Activation<<endl;
					cout<<tc::red <<"No Command Received"<<tc::none<<endl;

				}
				//			cout<<counter1<<": "<<2<<endl;

				//				console.SendCommand(cmdRecvd, arm, cmdValues, ctrlMode);

				//				if (arm<No_Gripper &&
				//						(cmdRecvd==ortosdata::CommandType::openGrip || cmdRecvd==ortosdata::CommandType::closeGrip))
				//				{
				//
				//				}
				//				if ( cmdRecvd==ortosdata::CommandType::holdMode)
				//				{
				//
				//				}
				//				if ( cmdRecvd==ortosdata::CommandType::stopArm){
				//					stopArmFlag[arm]=false;
				//					sysTimeGripper[arm]=time(0);
				//					sysTimeMSGripper[arm]=sysTimeGripper[arm]*1000;
				//				}

				//				obj_hri_control.hri_control_cmnd_flag[arm]=false;
				//				cout<<"************** 1 ****** "<<endl;
				//				obj_hri_control.publish_hri_control_cmnd(arm);
				cout<<"cmdRecvd:"<<cmdRecvd<<", arm:"<<arm<<endl;
				ack_counter[arm]=counter+1;
				if (cmdRecvd=="bimanCoordPos");
				else if (cmdRecvd=="singleArmTransportation")
					control_obj.cmdValues_control(cmdRecvd, arm,wTg);
				else if (cmdRecvd=="cartPos")
					control_obj.cmdValues_control(cmdRecvd, arm,cartValsArray);
				else if (cmdRecvd=="jointPos")
					control_obj.cmdValues_control(cmdRecvd, arm,jointValArray);
				else
				{
				//	cout<< "other Controller command types:: Not necessary to save last command"<<endl;
				}

			}


			control_obj.cmnd_recived_flag=true;
			cout<<"control_obj.youbot_control_msgs_vector.size(): "<<control_obj.youbot_control_msgs_vector.size()<<endl;
			control_obj.youbot_control_msgs_vector.clear();
			cout<<"control_obj.youbot_control_msgs_vector.size(): "<<control_obj.youbot_control_msgs_vector.size()<<endl;

		}
//		cout<<counter1<<": "<<3<<endl;
//		else if (input == "help") {
//				_lock.lock();
//				console.PrintCommandList();
//				_lock.unlock();
//		}
//		else {
//				_lock.lock();
//				cout << tc::red << "Error Parsing: " << console.ParsRetNames.at(parsRetVal) << tc::none << endl;
//				_lock.unlock();
//				obj_hri_control.publish_hri_control_cmnd(100);// 100: initialization
//		}
//	}
/*
	odh.ReadArm_wTt(l_wTt_v6Ctr, "/left");
	odh.ReadArm_wTt(r_wTt_v6Ctr, "/right");
	l_wTt_v6.CopyFrom(l_wTt_v6Ctr.d.data);
	r_wTt_v6.CopyFrom(r_wTt_v6Ctr.d.data);

	l_wTt = l_wTt_v6.Vect2TmatrixEsa();
	r_wTt = r_wTt_v6.Vect2TmatrixEsa();

	L_psi_ = CMAT::CartError( l_wTg,l_wTt  );
	R_psi_ = CMAT::CartError( r_wTg,r_wTt);


	 if ((L_psi_.GetFirstVect3().Norm(2) < angErrThresh_hri)
					&& (L_psi_.GetSecondVect3().Norm(2) < linErrThresh_hri))
			{
				flag = 1;
			}


           





*/
/*!
 * Acknowledge manager
 */       ret = odh.ReadVector6(reachinggoal,"mygoalreached");
            if (ret == ORTOS_RV_OK) {
                cout<<"reachinggoal is : "<<reachinggoal.d.data[0]<<endl;
                if(reachinggoal.d.data[0]==1)
	                control_obj.publish_control_ack(0,"baseControl");
                else if (reachinggoal.d.data[0]==2)
                    control_obj.publish_control_ack(0,"armControl");
                else if (reachinggoal.d.data[0]==3)
                    control_obj.publish_control_ack(0,"Gripper");
	
            }









		sysTime[0]=ros::Time::now();
//		sysTimeMS[0]=sysTime[0]*1000.0;//! msec
//		sysTimeMS[1]=sysTimeMS[0];
		sysTime[1]=sysTime[0];
		timeDiff[0]=(sysTime[0]-sysTimeGripper[0]).toSec();
		timeDiff[1]=(sysTime[1]-sysTimeGripper[1]).toSec();



//		cout<<"***********"<<endl;
//		cout<<"Gripper0:     "<<counter<<", "<<ack_counter[0]<<", "<< GripperFlag[0]<<", "<<timeDiff[0]<<endl;
//		cout<<"Gripper1:     "<<counter<<", "<<ack_counter[1]<<", "<< GripperFlag[1]<<", "<<timeDiff[1]<<endl;
//
//		cout<<"HoldingMode0: "<<counter<<", "<<ack_counter[0]<<", "<< HoldModeFlag[0]<<", "<<timeDiff[0]<<endl;
//		cout<<"HoldingMode1: "<<counter<<", "<<ack_counter[1]<<", "<< HoldModeFlag[1]<<", "<<timeDiff[1]<<endl;

		//! left arm hold mode command manager
		if (counter>ack_counter[0] && HoldModeFlag[0]==false ){
			//! time difference
			if(timeDiff[0]>0.01){
				HoldModeFlag[0]=true;
				control_obj.publish_control_ack(0,"HoldingMode");
			}
		}

		//! right arm hold mode command manager
		if (counter>ack_counter[1] && HoldModeFlag[1]==false ){

			//! time difference to change hold mode
			if(timeDiff[1]>0.01){
				HoldModeFlag[1]=true;
				control_obj.publish_control_ack(1,"HoldingMode");
			}
		}

		//! left gripper command manager
		//		else
		if (counter>ack_counter[0] && GripperFlag[0]==false){

			//! time to close or open gripper
			if(timeDiff[0]>0.8){
				GripperFlag[0]=true;
				control_obj.publish_control_ack(0,"Gripper");
			}
		}

		//! left gripper command manager
		//		else
		if(counter>ack_counter[1] && GripperFlag[1]==false){
			// time to close or open gripper
			if(timeDiff[1]>0.8){
				GripperFlag[1]=true;
				control_obj.publish_control_ack(1,"Gripper");
			}
		}
		if (stopArmFlag[0]==false ||	stopArmFlag[1]==false){
			//! left stop arm command manager
			if (counter>ack_counter[0] && stopArmFlag[0]==false ){
				//! time difference
				if(timeDiff[0]>0.01){
					stopArmFlag[0]=true;
					control_obj.publish_control_ack(0,"Stop");
				}
			}

			//! right stop arm command manager
			if (counter>ack_counter[1] && stopArmFlag[1]==false ){
				//! time difference to change hold mode
				if(timeDiff[1]>0.01){
					stopArmFlag[1]=true;
					control_obj.publish_control_ack(1,"Stop");
				}
			}
		}

		else{

			if( counter>ack_counter[0] && control_obj.control_ack_flag[0]==false)
			{
				arm=0;
				error_check= control_obj.control_goal_error_check(cmdRecvd, arm);

				if (error_check==true)
					control_obj.publish_control_ack(arm,"armControl");
			}

			if( counter>ack_counter[1] && control_obj.control_ack_flag[1]==false)
			{
				arm=1;
				error_check= control_obj.control_goal_error_check(cmdRecvd, arm);
				if (error_check==true)
					control_obj.publish_control_ack(arm,"armControl");
			}
			if( counter>ack_counter[2] && control_obj.control_ack_flag[2]==false)
			{
				arm=2;
				error_check= control_obj.control_goal_error_check(cmdRecvd, arm);
				if (error_check==true)
					control_obj.publish_control_ack(arm,"biManual");
			}
		}

		control_obj.publish_robotArm_Q();
		ros::spinOnce();
		task->WaitPeriod();
	}
	xcom->Release();
	task->Exit();
	ros::shutdown();
	return 0;
}

