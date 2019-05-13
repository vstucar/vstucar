# CMake generated Testfile for 
# Source directory: /home/garrus/ros/src/vstucar/car_core
# Build directory: /home/garrus/ros/src/vstucar/car_core/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_car_core_nosetests_tests.test_py_common.py "/home/garrus/ros/src/vstucar/car_core/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/garrus/ros/src/vstucar/car_core/cmake-build-debug/test_results/car_core/nosetests-tests.test_py_common.py.xml" "--return-code" "/opt/clion-2018.3.4/bin/cmake/linux/bin/cmake -E make_directory /home/garrus/ros/src/vstucar/car_core/cmake-build-debug/test_results/car_core" "/usr/bin/nosetests-2.7 -P --process-timeout=60 /home/garrus/ros/src/vstucar/car_core/tests/test_py_common.py --with-xunit --xunit-file=/home/garrus/ros/src/vstucar/car_core/cmake-build-debug/test_results/car_core/nosetests-tests.test_py_common.py.xml")
subdirs("gtest")
