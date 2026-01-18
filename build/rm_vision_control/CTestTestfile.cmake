# CMake generated Testfile for 
# Source directory: /home/nvidia/rm_robot/src/rm_vision_control
# Build directory: /home/nvidia/rm_robot/build/rm_vision_control
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rm_vision_control_nosetests_test "/home/nvidia/rm_robot/build/rm_vision_control/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/nvidia/rm_robot/build/rm_vision_control/test_results/rm_vision_control/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/nvidia/rm_robot/build/rm_vision_control/test_results/rm_vision_control" "/usr/bin/nosetests3 -P --process-timeout=60 --where=/home/nvidia/rm_robot/src/rm_vision_control/test --with-xunit --xunit-file=/home/nvidia/rm_robot/build/rm_vision_control/test_results/rm_vision_control/nosetests-test.xml")
set_tests_properties(_ctest_rm_vision_control_nosetests_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/nvidia/rm_robot/src/rm_vision_control/CMakeLists.txt;188;catkin_add_nosetests;/home/nvidia/rm_robot/src/rm_vision_control/CMakeLists.txt;0;")
subdirs("gtest")
