# CMake generated Testfile for 
# Source directory: /home/am/ws_robot/src/loam_velodyne
# Build directory: /home/am/ws_robot/build/loam_velodyne
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_loam_velodyne_rostest_test_loam.test "/home/am/ws_robot/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/am/ws_robot/build/test_results/loam_velodyne/rostest-test_loam.xml" "--return-code" "/usr/bin/python2 /opt/ros/melodic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/am/ws_robot/src/loam_velodyne --package=loam_velodyne --results-filename test_loam.xml --results-base-dir \"/home/am/ws_robot/build/test_results\" /home/am/ws_robot/build/loam_velodyne/test/loam.test ")
subdirs("src/lib")
