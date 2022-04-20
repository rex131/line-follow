# line-follow

# Part 1
## 1. Clone the source code
  cd ~/catkin_ws/src
  
  git clone git@github.com:zhaojieting/lane_following.git
  
## 2. Catkin make the lane following package
  cd ..
  
  catkin_make

## 3. Add course models
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models
   
## 4. Launch the gazebo map
   source ~/catkin_ws/devel/setup.bash
   
   roslaunch lane_following race_track.launch 

## 5. Run lane following python node
   
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash
   
   rosrun lane_following lane_following.py

# Part 2

## Run lane following python node
   
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following2.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash
   
   rosrun lane_following lane_following2.py
   
   # Part 3
   ## Run lane following python node
   
   cd ~/catkin_ws/src/lane_following/scripts/
   
   chmod +x lane_following3.py
   
   cd ~/catkin_ws
   
   source devel/setup.bash
   
   rosrun lane_following lane_following3.py

