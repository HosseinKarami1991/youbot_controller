/*
 * interface3.cc
 *
 *  Created on: Dec 2, 2016
 *      Author: nasa
 */



//CPP
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <control_youbot/roc.h>
#include <time.h>
#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string/erase.hpp>

#include <control_youbot/ortosdata_defines.h>
//#include "command_consoleAPI.h"
#include "youbotcmd_msgs/ControlYoubot.h"
#include "youbotcmd_msgs/ControlYoubotGoalReachAck.h"
#include "control_youbot/roc.h"

using namespace std;


void CommnaderParser(string input, youbotcmd_msgs::ControlYoubot& Controlmsg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "COMMANDER_API_HRI_CONTROL_YOUBOT");
	ros::NodeHandle nh;
	ros::Rate loop_rate(2);

	/*
	 * Example Commad type:
	 * tasnportSingleArm , [armName] , [wTo] , [wTg] ==> trasnportSingleArm , left , 1.1 1.2 1.3 1.4 1.5 1.6 , 2.1 2.2 2.3 2.4 2.4 2.6
	 * tasnportBimanArm , [armName Leader] [armName Follower], [wTo] , [wTg] ==> trasnportBimanArm , left right, 1.1 1.2 1.3 1.4 1.5 1.6 , 2.1 2.2 2.3 2.4 2.4 2.6
	 *
	 * */

	string input;
	/// Copy all commander arguments into a string, except from executable name
	for (int i = 1; i < argc; i++) {
		input = input + argv[i];
		if (i < argc - 1)
			input = input + " ";
	}
	cout<<"Input: "<<input<<endl;
	youbotcmd_msgs::ControlYoubot control_msg;

	ros::Publisher publishControlCommand=nh.advertise<youbotcmd_msgs::ControlYoubot>("youbot_control_command",80);

	CommnaderParser(input, control_msg);

//	vector<float> goalPose;
//	for(int i=0;i<6;i++)
//		goalPose.push_back(0.5+(float)i);
//	control_msg.Activation=0;
//	control_msg.oneArm.armIndex=0;
//	control_msg.oneArm.armCmndType="cartPos";
//	for(int i=0;i<6;i++)
//		control_msg.oneArm.cartGoal.cartesianPosition[i]=goalPose[i];
//	publishControlCommand.publish(control_msg);

	int count=0;
	while(ros::ok())
	{
		count++;
		control_msg.youbotArm.cartGoal.cartesianPosition[5]=(float)count;
//		cout<<publishControlCommand.isLatched()<<endl;
		if(count==2)
		{
			publishControlCommand.publish(control_msg);
			int actNo=(int)control_msg.Activation;
//			cout<<publishControlCommand.isLatched()<<endl;
//			ros::spinOnce();
//			loop_rate.sleep();

			ROS_INFO("publish control msg done!");
//			usleep(0.1e6);
//			break;
		}

		if(count==3){	exit(1); }
		ros::spinOnce();
		loop_rate.sleep();
	}
//	ros::shutdown();
	return 1;
}
//***********************************************************************************
//***********************************************************************************

void CommnaderParser(string input, youbotcmd_msgs::ControlYoubot& Controlmsg){
	vector<string> inputMsg;
	boost::split( inputMsg, input, boost::is_any_of(","));

	string cmndName=inputMsg[0];
	boost::erase_all(cmndName, " ");

	vector<string> armsStr, armsStrTemp;
	boost::split( armsStrTemp, inputMsg[1], boost::is_any_of(" "));
	for(int i=0;i<armsStrTemp.size();i++)
		if(armsStrTemp[i]!=" ")
			armsStr.push_back(armsStrTemp[i]);

	vector<string> wToStr, wToStrTemp;
	boost::split( wToStr, inputMsg[2], boost::is_any_of(" "), boost::token_compress_on);
//	for(int i=0;i<wToStrTemp.size();i++)
//		if(wToStrTemp[i]!=" ")
//			wToStr.push_back(wToStrTemp[i]);


	vector<string> wTgStr,wTgStrTemp;
	boost::split( wTgStr, inputMsg[3], boost::is_any_of(" "), boost::token_compress_on);
//	for(int i=0;i<wTgStr.size();i++)
//		if(wTgStrTemp[i]!=" ")
//			wTgStr.push_back(wTgStrTemp[i]);



	vector<float> wTo,wTg;
	float val;
	std::string::size_type sz;
	char* pEnd;
	for(int i=0;i<6; i++){
		cout<<wToStr[i]<< " "<<wTgStr[i]<<",";
		val=atof(wToStr[i].c_str());
		wTo.push_back(val);

		val=atof(wTgStr[i].c_str());
		wTg.push_back(val);
	}
	cout<<endl;

	vector<int> arms;
	for(int i=0;i<armsStrTemp.size(); i++){
		if(armsStrTemp[i]=="left")
			arms.push_back(0);
		else if(armsStrTemp[i]=="right")
			arms.push_back(1);
		else
		{
			cout<<"Error in arms parameter: "<<armsStr[i]<<endl;
			exit(1);
		}
	}

	if(cmndName=="trasnportSingleArm")
	{
		Controlmsg.Activation=5;
		Controlmsg.youbotArm.youbotIndex=arms[0];
		if((int)arms.size()!=1)
			cout<<"Error: arm size is not one: "<<arms.size()<<endl;

		for(int i=0;i<6;i++)
		{
			Controlmsg.youbotArm.wTo.cartesianPosition[i]=wTo[i];
			Controlmsg.youbotArm.wTg.cartesianPosition[i]=wTg[i];
		}
	}
	else if(cmndName=="trasnportBimanArm")
	{
		Controlmsg.Activation=1;
		Controlmsg.youbotArm.youbotIndex=arms[0];
		Controlmsg.youbotArm.youbotIndex=arms[1];
		if((int)arms.size()!=2)
			cout<<"Error: arm size is not two: "<<arms.size()<<endl;

		for(int i=0;i<6;i++)
		{
			Controlmsg.youbotArm.wTo.cartesianPosition[i]=wTo[i];
			Controlmsg.youbotArm.wTg.cartesianPosition[i]=wTg[i];

		}
	}
	else
	{
		cout<<"Error: command not implemented:"<<cmndName<<"."<<endl;
	}




};

