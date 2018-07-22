rm ros/src/twist_controller/*.csv
source build.sh
source ros/devel/setup.sh
roslaunch ros/src/twist_controller/launch/dbw_test.launch
