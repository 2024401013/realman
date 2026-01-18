# Install script for directory: /home/nvidia/rm_robot/src/rm_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nvidia/rm_robot/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/rm_robot/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/rm_robot/install" TYPE PROGRAM FILES "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/rm_robot/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/rm_robot/install" TYPE PROGRAM FILES "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/rm_robot/install/setup.bash;/home/nvidia/rm_robot/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/rm_robot/install" TYPE FILE FILES
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/setup.bash"
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/rm_robot/install/setup.sh;/home/nvidia/rm_robot/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/rm_robot/install" TYPE FILE FILES
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/setup.sh"
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/rm_robot/install/setup.zsh;/home/nvidia/rm_robot/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/rm_robot/install" TYPE FILE FILES
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/setup.zsh"
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/rm_robot/install/setup.fish;/home/nvidia/rm_robot/install/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/rm_robot/install" TYPE FILE FILES
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/setup.fish"
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/rm_robot/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/rm_robot/install" TYPE FILE FILES "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs/msg" TYPE FILE FILES
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Arm_Analog_Output.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Arm_Digital_Output.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Arm_Joint_Speed_Max.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Arm_Software_Version.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Arm_IO_State.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/JointPos.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/MoveC.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/MoveJ.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/MoveJ_P.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/MoveL.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Tool_Analog_Output.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Tool_Digital_Output.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Tool_IO_State.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Plan_State.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Cabinet.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/ChangeTool_State.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/ChangeTool_Name.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/ChangeWorkFrame_State.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/ChangeWorkFrame_Name.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Arm_Current_State.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/GetArmState_Command.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Stop.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_Teach.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Pos_Teach.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Ort_Teach.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Stop_Teach.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Gripper_Set.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Gripper_Pick.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_Enable.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_Max_Speed.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/IO_Update.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Turtle_Driver.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Socket_Command.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Start_Multi_Drag_Teach.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Set_Force_Position.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Force_Position_Move_Joint.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Force_Position_Move_Pose.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Force_Position_State.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Six_Force.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Manual_Set_Force_Pose.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/CartePos.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Lift_Height.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Lift_Speed.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_Current.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_Step.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/ArmState.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Hand_Posture.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Hand_Seq.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Hand_Speed.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Hand_Force.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Hand_Angle.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Hand_Status.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/LiftState.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Servo_GetAngle.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Servo_Move.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Set_Realtime_Push.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Arm_Current_Status.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_En_Flag.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_PoseEuler.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_Speed.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_Temperature.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Joint_Voltage.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Lift_In_Position.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/CartePosCustom.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/JointPosCustom.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Force_Position_Move_Pose_Custom.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/RS485_Mode.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Set_Modbus_Mode.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Register_Data.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Read_Register.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Err.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Rm_Plus_Base.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Rm_Plus_State.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Write_Register.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Softwarebuildinfo.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Moveloffset.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Trajectoryinfo.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Trajectorylist.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Gettrajectorylist.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Programrunstate.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Flowchartrunstate.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Jointversion.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Mastername.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Modbusreaddata.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Modbustcpmasterinfo.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Modbustcpmasterlist.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/UpdateTCPmasterparam.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Get_TCP_Master_List_Param.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/RS485params.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Read_TCPandRTU.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Write_TCPandRTU.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Read_ModbusRTU.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Write_ModbusRTU.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Expand_In_Position.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Expand_Position.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/Expand_Speed.msg"
    "/home/nvidia/rm_robot/src/rm_msgs/msg/ExpandState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs/cmake" TYPE FILE FILES "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/rm_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nvidia/rm_robot/devel/.private/rm_msgs/include/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/nvidia/rm_robot/devel/.private/rm_msgs/share/roseus/ros/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/nvidia/rm_robot/devel/.private/rm_msgs/share/common-lisp/ros/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/nvidia/rm_robot/devel/.private/rm_msgs/share/gennodejs/ros/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/nvidia/rm_robot/devel/.private/rm_msgs/lib/python3/dist-packages/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/nvidia/rm_robot/devel/.private/rm_msgs/lib/python3/dist-packages/rm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/rm_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs/cmake" TYPE FILE FILES "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/rm_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs/cmake" TYPE FILE FILES
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/rm_msgsConfig.cmake"
    "/home/nvidia/rm_robot/build/rm_msgs/catkin_generated/installspace/rm_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rm_msgs" TYPE FILE FILES "/home/nvidia/rm_robot/src/rm_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/nvidia/rm_robot/build/rm_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nvidia/rm_robot/build/rm_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
