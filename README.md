cd ros_ws  
sudo apt-get install ros-kinetic-joy  
run "catkin_make" make sure there are no errors.  
edit ~/.bashrc, add "source /home/..../rosws/devel/setup.bash" to the end of the .bashrc file.  
close the terminal and reopen the terminal  
run roscore  
run vrep, open scenes/assignment6.ttt  
. ~/ros-ws/devel/setup.bash  
rosrun state_estimation state_estimation  
rosrun rqt_plot rqt_plot #add 3 /x 3 /y values  
