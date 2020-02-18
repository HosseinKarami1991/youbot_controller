/*

 * hri_control_Class.cpp
 *
 *  Created on: Jun 20, 2016
 *      Author: Kourosh Darvish
 */

#include "api_control_class_youbot.hpp"

api_control_class_youbot::api_control_class_youbot(){
	//arm left::0    arm right::1
	cout<<">>>>>>>>>>>>>>>>>>>>>>>>>HRI_CONTROL_BAXTER:   "<<endl;
	cmnd_recived_flag=true;
//	hri_control_msg			="0";
    mocappos_.resize(6);
	sub_hri_control_cmnd	= nh.subscribe("youbot_control_command", 80, &api_control_class_youbot::Callback_control, this);
	sub_mocap	= nh.subscribe("object_ctrl_1", 80, &api_control_class_youbot::Callback_mocap, this);
	pub_robot_control_ack	=nh.advertise<youbotcmd_msgs::ControlYoubotGoalReachAck>("youbot_control_ack",80);
	pub_ArmLeftQ	=nh.advertise<std_msgs::Float64MultiArray>("Q_youbotarm",1);


	jointValsArr=new double *[Arm_states];
	cartValsArray=new double *[Arm_states];
	for (int i=0;i<Arm_states;i++){	jointValsArr[i]=new double [num_joints];}
	for (int i=0;i<Arm_states;i++){	cartValsArray[i]=new double [6];}

	for (int i=0;i<Arm_states;i++){
		hri_control_cmnd_flag[i]=true;
		control_ack_flag[i]=true;
		flag[i]=0;
		timeCounter[i]=0;
		for (int j=0;j<num_joints;j++)
			jointValsArr[i][j]=0.0;
		for (int j=0;j<6;j++){
			cartValsArray[i][j]=0.0;
			toolValsArray[j]=0.0;
		}
	}
	 qErrThresh_hri=0.001*10, angErrThresh_hri=3 * M_PI / 180.0, linErrThresh_hri=0.015;
}

void api_control_class_youbot::Callback_control(const youbotcmd_msgs::ControlYoubot& msg1) {

	cout<<FGRN(BOLD("api_control_class_youbot::Callback_control: command received "))<<endl;
	ROS_INFO("Control command received");
	cmnd_recived_flag=false;
	youbot_control_msg=msg1;
	youbot_control_msgs_vector.push_back(youbot_control_msg);
	int activationNumber=youbot_control_msg.Activation;
	cout<<"command Activation Value: "<<activationNumber<<endl;
//	if (a1==4)
//		cout<<"hri_control_msg.holdModeArm.arm: "<<a2<<endl;
	if (activationNumber==0){
		int a3=youbot_control_msg.youbotArm.youbotIndex;
		string cmndtype= youbot_control_msg.youbotArm.youbotCmdType;
		if(cmndtype=="cartPos")
		{
			cout<<"cartesianPoseGoal, arm index: " <<a3<<", pose: ";
			for(int i=0;i<6;i++)
				cout<<youbot_control_msg.youbotArm.cartGoal.cartesianPosition[i]<<" ";
			cout<<endl;
		}
		else if(cmndtype=="jointPos")
		{
			cout<<"JointPoseGoal, arm index: " <<a3<<", pose: ";
			for(int i=0;i<5;i++)
				cout<<youbot_control_msg.youbotArm.cartGoal.youbotJointPose[i]<<" ";
			cout<<endl;
		}
		else
		{
			cout<<"Robot command type is not defined: "<<cmndtype<<endl;
		}
	}
	else if (activationNumber==1){
		
	}
	else if(activationNumber==2)
	{
		cout<<"Gripping command: arm: "<<int(youbot_control_msg.youbotArm.youbotIndex)<<" , value: "<<bool (youbot_control_msg.youbotArm.value)<<endl;
	}
	else if(activationNumber==3)
	{
		cout<<"Stop command: arm: "<<int(youbot_control_msg.youbotArm.youbotIndex)<<endl;

	}
	else if(activationNumber==4)
	{
		cout<<"holdMode command: arm: "<<int(youbot_control_msg.youbotArm.youbotIndex)<<endl;
	}
	else if(activationNumber==5)
	{
		cout<<"basepositioncommand command: arm: "<<int(youbot_control_msg.youbotArm.youbotIndex)<<endl;
	}
	
	else
	{
		cout<<"Error In arriving msg"<<activationNumber<<endl;
	}



}
void api_control_class_youbot::Callback_mocap(const mocap_msgs::mocapvector::ConstPtr& mocvec)

{
	//cout<<"**************"<<mocvec<<endl;
	// cout<<"size is "<<mocvec->objects.size()<<endl;
 
   // int goalindex=knowledge_msg.response.pose[];
   
    //next object pos 
   

	if(objindex_==10){
       	for (int i=0; i<mocvec->objects.size(); ++i)

    {
     
    	const  mocap::GetPoseFromMocapMsg &obj = mocvec->objects[i];
    	if(obj.index==0){
    
            mocappos_[0]=obj.mocap.yaw;
            mocappos_[1]=obj.mocap.x;
            mocappos_[2]=obj.mocap.y;
            
 
    		
    	}
    	else if(obj.index==1){
            mocappos_[3]=0.0;
            mocappos_[4]=1.1;
            mocappos_[5]=-0.55;
            
  
    		
    	}

        
      // cout<<"x: " << obj.mocap.y << "y: " << obj.mocap.x<<endl;
     // ROS_INFO_STREAM("UL: " << obj.mocap.y << "UR: " << obj.mocap.x);
                      
    
}







	}
	else{



	for (int i=0; i<mocvec->objects.size(); ++i)

    {
     
    	const  mocap::GetPoseFromMocapMsg &obj = mocvec->objects[i];
    	if(obj.index==0){
    
    
            mocappos_[1]=obj.mocap.x;
            mocappos_[2]=obj.mocap.y;
            mocappos_[0]=obj.mocap.yaw;
 
    		
    	}
    	else if(obj.index==objindex_){
  
            mocappos_[4]=obj.mocap.x;
            mocappos_[5]=obj.mocap.y;
            mocappos_[3]=obj.mocap.yaw;
  
    		
    	}

        
      // cout<<"x: " << obj.mocap.y << "y: " << obj.mocap.x<<endl;
     // ROS_INFO_STREAM("UL: " << obj.mocap.y << "UR: " << obj.mocap.x);
                      
    
}













	}


}


void api_control_class_youbot::publish_control_cmnd(int arm_index){

//	cout<<">>>>>>>>>>>>>>>>>>>>>>>>  hri_control_publish_cmnd!" <<endl;
//
//			std_msgs::String msg_hri_control_ack;
//			if (arm_index==0)
//				msg_hri_control_ack.data="hri_control_command_recieved_left";
//			else if (arm_index==1)
//				msg_hri_control_ack.data="hri_control_command_recieved_right";
//			else if (arm_index==2)
//				msg_hri_control_ack.data="hri_control_command_recieved_bimanual";
//			else if (arm_index==100)
//				msg_hri_control_ack.data="hri_control_command_Initialization";
//
//			if (arm_index<Arm_states)
//			{
//				hri_control_cmnd_flag[arm_index]=true;
//				control_ack_flag[arm_index]=false;
//			}
//			ROS_INFO("I publish hri_control command: %s",msg_hri_control_ack.data.c_str());
//			pub_robot_control_ack.publish(msg_hri_control_ack);

}

void api_control_class_youbot::publish_control_ack(int arm_index, string ack_type){

	cout<<">>>>>>>>>>>>>>>>>>>>>>>>  hri_control_publish_ack!" <<endl;

	controlAck.armState=arm_index;

	if (ack_type=="armControl") controlAck.ctrlCmndTypeAck=2;
	if (ack_type=="baseControl") 	controlAck.ctrlCmndTypeAck=1;
	if (ack_type=="Gripper") 	controlAck.ctrlCmndTypeAck=3;
	if (ack_type=="Stop") 		controlAck.ctrlCmndTypeAck=4;
	if (ack_type=="HoldingMode")controlAck.ctrlCmndTypeAck=5;
	control_ack_flag[arm_index]=true;

//	std_msgs::String msg_hri_control_ack;
//		msg_hri_control_ack.data="0";
//		if (hri_control_cmnd_flag[arm_index]==true)
//		{
//			if (arm_index==0)
//				msg_hri_control_ack.data="GoalReachedLeft";
//			if (arm_index==1)
//				msg_hri_control_ack.data="GoalReachedRight";
//			if (arm_index==2)
//				msg_hri_control_ack.data="GoalReachedBiManual";
//			hri_control_ack_flag[arm_index]=true;
//		}



		ROS_INFO("Control acknowledge is published: arm:%d, cmndType: %d",controlAck.armState,controlAck.ctrlCmndTypeAck);
		pub_robot_control_ack.publish(controlAck);

}



void api_control_class_youbot::publish_robotArm_Q(void){

	vector<double> _q_(num_joints, 0.0);
	std_msgs::Float64MultiArray q_msg;
	q_msg.data.resize(num_joints);

//! read left arm q and publish q:
	odh.ReadArmFeedback(_fbk_,ortosdata::topicnames::left);
	_q_.assign(_fbk_.d.q, _fbk_.d.q + num_joints);
	for (int i=0;i<num_joints;i++)
		q_msg.data[i]=_q_.at(i);
	pub_ArmLeftQ.publish(q_msg);
//! read right arm q and publish q:

}





void api_control_class_youbot::cmdValues_control( string cmd,const int arm, double *cmdValues){

	if (cmd=="jointPos"){

			jointValsArr[arm][0]=cmdValues[0];jointValsArr[arm][1]=cmdValues[1];jointValsArr[arm][2]=cmdValues[2];
			jointValsArr[arm][3]=cmdValues[3];jointValsArr[arm][4]=cmdValues[4];
	}
	else if (cmd=="cartPos"){
		cout<<"cmdValues_hri_control"<<"cartPos"<<arm<<endl;
		cartValsArray[arm][0]=cmdValues[0];cartValsArray[arm][1]=cmdValues[1];cartValsArray[arm][2]=cmdValues[2];
		cartValsArray[arm][3]=cmdValues[3];cartValsArray[arm][4]=cmdValues[4];cartValsArray[arm][5]=cmdValues[5];
	}

}


bool api_control_class_youbot::control_goal_error_check(string cmd, const int arm){
	bool return_value=false;
	flag[arm] = 0;
	double sum;
//	cout<<"goal_error_hri_control: "<<cmd<<" "<<arm<<endl;
			if ( cmd=="jointPos"){
//				vector<double> l_q(num_joints, 0.0), l_q_dot(num_joints, 0.0);
//				vector<double> r_q(num_joints, 0.0), r_q_dot(num_joints, 0.0);

				vector<double> _q_(num_joints, 0.0), _q_dot(num_joints, 0.0);

				if (arm==0)
					odh.ReadArmFeedback(_fbk_,ortosdata::topicnames::left);
				else if (arm==1)
					odh.ReadArmFeedback(_fbk_,ortosdata::topicnames::right);
				_q_.assign(_fbk_.d.q, _fbk_.d.q + num_joints);

				flag[arm] = 1;
				for (int i=0;i<num_joints;i++){
						if (std::fabs(_q_.at(i)-jointValsArr[arm][i])>qErrThresh_hri )
							flag[arm] = 0;
				}
				if (flag[arm] == 1)
					++timeCounter[arm];
				else
					timeCounter[arm] = 0;

				if (timeCounter[arm] > errThreshDuration)
				{
					timeCounter[arm] = 0;
					return_value = true;
				}
			}
			else if ( cmd=="cartPos"){
//				cout<<"cartPos 1: "<<endl;
				CMAT::Vect6  l_wTg_v6(const_cast<double*>(cartValsArray[0]));
				CMAT::Vect6  r_wTg_v6(const_cast<double*>(cartValsArray[1]));
				l_wTg = l_wTg_v6.Vect2TmatrixEsa();
				r_wTg = r_wTg_v6.Vect2TmatrixEsa();

				odh.ReadArm_wTt(l_wTt_v6Ctr, ortosdata::topicnames::left);
				odh.ReadArm_wTt(r_wTt_v6Ctr, ortosdata::topicnames::right);
				l_wTt_v6.CopyFrom(l_wTt_v6Ctr.d.data);
				r_wTt_v6.CopyFrom(r_wTt_v6Ctr.d.data);

				l_wTt = l_wTt_v6.Vect2TmatrixEsa();
				r_wTt = r_wTt_v6.Vect2TmatrixEsa();

				L_psi_ = CMAT::CartError( l_wTg,l_wTt  );
				R_psi_ = CMAT::CartError( r_wTg,r_wTt);

//				cout<<"cartPos L 1: "<<L_psi_.GetFirstVect3().Norm(2)<<" "<<L_psi_.GetSecondVect3().Norm(2)<<endl;
//				cout<<"cartPos R 1: "<<R_psi_.GetFirstVect3().Norm(2)<<" "<<R_psi_.GetSecondVect3().Norm(2)<<endl;

				if ((L_psi_.GetFirstVect3().Norm(2) < angErrThresh_hri)
						&& (L_psi_.GetSecondVect3().Norm(2) < linErrThresh_hri))
				{
//					cout<<"cartPos L 2: "<<L_psi_.GetFirstVect3().Norm(2)<<" "<<L_psi_.GetSecondVect3().Norm(2)<<endl;
					flag[0] = 1;
				}
				if ((R_psi_.GetFirstVect3().Norm(2) < angErrThresh_hri)
								&& (R_psi_.GetSecondVect3().Norm(2) < linErrThresh_hri))
				{
//					cout<<"cartPos R 2: "<<R_psi_.GetFirstVect3().Norm(2)<<" "<<R_psi_.GetSecondVect3().Norm(2)<<endl;
					flag[1] = 1;
				}

				if (flag[arm] == 1)
					++timeCounter[arm];
				else
					timeCounter[arm] = 0;

				if (timeCounter[arm] >errThreshDuration)
				{
					timeCounter[arm] = 0;
					return_value = true;
				}
			}
			else if ( cmd=="bimanCoordPos"){
				flag[0] = 0;flag[1] = 0;

//				CMAT::Vect6  l_wTg_v6(const_cast<double*>(cartValsArray[0]));
//				CMAT::Vect6  r_wTg_v6(const_cast<double*>(cartValsArray[1]));
//				l_wTg = l_wTg_v6.Vect2TmatrixEsa();
//				r_wTg = r_wTg_v6.Vect2TmatrixEsa();

				odh.ReadArm_wTt(l_wTt_v6Ctr, ortosdata::topicnames::left);
				odh.ReadArm_wTt(r_wTt_v6Ctr, ortosdata::topicnames::right);
				l_wTt_v6.CopyFrom(l_wTt_v6Ctr.d.data);
				r_wTt_v6.CopyFrom(r_wTt_v6Ctr.d.data);

				l_wTt = l_wTt_v6.Vect2TmatrixEsa();
				r_wTt = r_wTt_v6.Vect2TmatrixEsa();

				L_psi_ = CMAT::CartError( l_wTg,l_wTt  );
				R_psi_ = CMAT::CartError( r_wTg,r_wTt);

//				cout<<L_psi_(1)<<" "<<L_psi_(2)<<" "<<L_psi_(3)<<" "<<L_psi_(4)<<" "<<L_psi_(5)<<" "<<L_psi_(6)<<", ";
//				cout<<R_psi_(1)<<" "<<R_psi_(2)<<" "<<R_psi_(3)<<" "<<R_psi_(4)<<" "<<R_psi_(5)<<" "<<R_psi_(6)<<", ";
//				cout<<l_wTg(1)<<" "<<l_wTg(2)<<" "<<l_wTg(3)<<" "<<l_wTg(4)<<" "<<l_wTg(5)<<" "<<l_wTg(6)<<", ";
//				cout<<r_wTg(1)<<" "<<r_wTg(2)<<" "<<r_wTg(3)<<" "<<r_wTg(4)<<" "<<r_wTg(5)<<" "<<r_wTg(6)<<", ";
//				cout<<l_wTt(1)<<" "<<l_wTt(2)<<" "<<l_wTt(3)<<" "<<l_wTt(4)<<" "<<l_wTt(5)<<" "<<l_wTt(6)<<", ";
//				cout<<r_wTt(1)<<" "<<r_wTt(2)<<" "<<r_wTt(3)<<" "<<r_wTt(4)<<" "<<r_wTt(5)<<" "<<r_wTt(6)<<endl;

//				cout<<"cartPos L 1: "<<L_psi_.GetFirstVect3().Norm(2)<<" "<<L_psi_.GetSecondVect3().Norm(2)<<endl;
//				cout<<"cartPos R 1: "<<R_psi_.GetFirstVect3().Norm(2)<<" "<<R_psi_.GetSecondVect3().Norm(2)<<endl;

				if ((L_psi_.GetFirstVect3().Norm(2) < angErrThresh_hri)
					&& (L_psi_.GetSecondVect3().Norm(2) < linErrThresh_hri))
				{
//					cout<<"cartPos L 2: "<<L_psi_.GetFirstVect3().Norm(2)<<" "<<L_psi_.GetSecondVect3().Norm(2)<<endl;
					flag[0] = 1;
				}
				if ((R_psi_.GetFirstVect3().Norm(2) < angErrThresh_hri)
						&& (R_psi_.GetSecondVect3().Norm(2) < linErrThresh_hri))
				{
//					cout<<"cartPos R 2: "<<R_psi_.GetFirstVect3().Norm(2)<<" "<<R_psi_.GetSecondVect3().Norm(2)<<endl;
					flag[1] = 1;
				}

				if (flag[0] == 1 )//&& flag[1] == 1
					++timeCounter[arm];
				else
					timeCounter[arm] = 0;

				if (timeCounter[arm] >errThreshDuration)
				{
					timeCounter[arm] = 0;
					return_value = true;
				}

			}
			else if ( cmd=="singleArmTransportation"){

				CMAT::Vect6  l_wTg_v6(const_cast<double*>(cartValsArray[0]));
				CMAT::Vect6  r_wTg_v6(const_cast<double*>(cartValsArray[1]));
				l_wTg = l_wTg_v6.Vect2TmatrixEsa();
				r_wTg = r_wTg_v6.Vect2TmatrixEsa();

				odh.ReadArm_wTt(l_wTt_v6Ctr, ortosdata::topicnames::left);
				odh.ReadArm_wTt(r_wTt_v6Ctr, ortosdata::topicnames::right);
				l_wTt_v6.CopyFrom(l_wTt_v6Ctr.d.data);
				r_wTt_v6.CopyFrom(r_wTt_v6Ctr.d.data);

				l_wTt = l_wTt_v6.Vect2TmatrixEsa();
				r_wTt = r_wTt_v6.Vect2TmatrixEsa();

				L_psi_ = CMAT::CartError( l_wTg,l_wTt  );
				R_psi_ = CMAT::CartError( r_wTg,r_wTt);

//				cout<<"*******"<<endl;
//				cout<<"l_wTg: "<<l_wTg.Tmatrix2Vect().GetFirstVect3()(1)<<" "<<l_wTg.Tmatrix2Vect().GetFirstVect3()(2)<<" "<<l_wTg.Tmatrix2Vect().GetFirstVect3()(3)<<", "<<l_wTg.Tmatrix2Vect().GetSecondVect3()(1)<<" "<<l_wTg.Tmatrix2Vect().GetSecondVect3()(2)<<" "<<l_wTg.Tmatrix2Vect().GetSecondVect3()(3)<<" "<<endl;
//				cout<<"l_wTt: "<<l_wTt.Tmatrix2Vect().GetFirstVect3()(1)<<" "<<l_wTt.Tmatrix2Vect().GetFirstVect3()(2)<<" "<<l_wTt.Tmatrix2Vect().GetFirstVect3()(3)<<", "<<l_wTt.Tmatrix2Vect().GetSecondVect3()(1)<<" "<<l_wTt.Tmatrix2Vect().GetSecondVect3()(2)<<" "<<l_wTt.Tmatrix2Vect().GetSecondVect3()(3)<<" "<<endl;

//				cout<<"LeftArm Error: "<<L_psi_.GetFirstVect3().Norm(2)<<" "<<L_psi_.GetSecondVect3().Norm(2)<<endl;

				if ((L_psi_.GetFirstVect3().Norm(2) < angErrThresh_hri)
						&& (L_psi_.GetSecondVect3().Norm(2) < linErrThresh_hri))
				{
					flag[0] = 1;
				}
				if ((R_psi_.GetFirstVect3().Norm(2) < angErrThresh_hri)
						&& (R_psi_.GetSecondVect3().Norm(2) < linErrThresh_hri))
				{
					flag[1] = 1;
				}

				if (flag[arm] == 1)
					++timeCounter[arm];
				else
					timeCounter[arm] = 0;

				if (timeCounter[arm] >errThreshDuration)
				{
					timeCounter[arm] = 0;
					return_value = true;
				}


			}
			else if ( cmd=="stopArm"){
			}
			else if ( cmd=="openGrip"){
			}
			else if ( cmd=="closeGrip"){
			}
			else if ( cmd=="holdMode"){
			}
			else{
				cout<<"No defined command type for goal error check"<<endl;
			}

return return_value;
}

api_control_class_youbot::~api_control_class_youbot(){

	for (int i=0;i<Arm_states;i++)
		delete [] jointValsArr[i];
	delete [] jointValsArr;
}



