cmake_minimum_required(VERSION 2.8.3)
project(simYoubot_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs )

add_message_files(
  DIRECTORY msg
  FILES
  transformation.msg
  sim_singleYoubot.msg
  sim_youbot.msg
  sim_singleMoveCartPose.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  simulateYoubotSRV.srv)


generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs  )

