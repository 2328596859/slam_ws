cmake_minimum_required(VERSION 3.0.2)
project(qyh_nav_ctrl_msg)

set(PACKAGE_DEPENDENCIES
  geometry_msgs
  std_msgs
)
# qyh_nav_ctrl_msg/NAV_POINT[] points
find_package(catkin REQUIRED COMPONENTS message_generation ${PACKAGE_DEPENDENCIES})

add_message_files(
  FILES
    NAV_POINT.msg
    NAV_MAP_POINTS.msg
    NAV_STATUS.msg
    NAV_TRAJ.msg
    NAV_MAP_TRAJS.msg
    NAV_TIME_TASK.msg
    NAV_MAP_TIME_TASKS.msg
)

add_service_files(
  FILES
    SERVICE_ADD_LOOP_POINT.srv    
    SERVICE_ADD_LOOP_POINT_NAME.srv    
    SERVICE_DELETE_POINT.srv
    SERVICE_LIST_ALL_MAP_POINT.srv
    SERVICE_ADD_POINT.srv         
    SERVICE_LIST_POINT.srv
    SERVICE_GET_CHARGE_POINT.srv  
    SERVICE_MODIFY_POINT.srv
    SERVICE_GET_START_POINT.srv   
    SERVICE_GET_TAG_RESET_POINT.srv
    SERVICE_SET_CHARGE_POINT.srv
    SERVICE_GO_POINT.srv          
    SERVICE_SET_START_POINT.srv
    SERVICE_GO_TEMP_POINT.srv
    SERVICE_ADD_TRAJ.srv    
    SERVICE_DELETE_TRAJ.srv   
    SERVICE_MODIFY_TRAJ.srv
    SERVICE_LIST_TRAJ.srv
    SERVICE_LIST_ALL_MAP_TRAJ.srv
    SERVICE_GO_TRAJ.srv  
    SERVICE_LIST_DEVICE_CMD.srv  
    SERVICE_ADD_TIME_TASK.srv
    SERVICE_DELETE_TIME_TASK.srv
    SERVICE_LIST_ALL_MAP_TIME_TASK.srv
    SERVICE_LIST_TIME_TASK.srv
    SERVICE_MODIFY_TIME_TASK.srv
    SERVICE_SET_TAG_RESET_POINT.srv
)

generate_messages(
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
)
catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
)
