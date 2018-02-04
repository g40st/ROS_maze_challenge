# Maze Solver (ROS)
This repository enables a robot to leave a maze. Basically it uses wall detection and wall follow algorithms. 

<img src="https://user-images.githubusercontent.com/7523395/35778687-047ef1e4-09c2-11e8-8766-cceff7a3e3f9.png" width="50%">

## Installing / Getting started (Linux)

### Install Ros and other Tools

**Warning! Please use Ubuntu 16.04 as this is the Long Term Support Version (LTS). If you are using Ubuntu 17.04 you have to use ROS Lunar release on your own risk!**

First goto http://wiki.ros.org/kinetic/Installation/Ubuntu and follow the instructions there. The installation can take some time depending on your internet connection.

After the ros setup is completed you should install these extra tools

```
sudo apt-add-repository ppa:webupd8team/atom
sudo apt-get update
sudo apt-get install git gitg htop terminator atom
sudo apt-get install ros-kinetic-turtlebot-simulator ros-kinetic-turtlebot-teleop
```

### Create catkin workspace

All your ROS packages must be stored in a special folder called __catkin workspace__.
ROS uses the [catkin](http://docs.ros.org/api/catkin/html/) build system to manage your codebase.
We will not go deeper into exactly what this build system is, but for now, you need to create a catkin workspace before you create your first package.

__This is very important! All your ROS code must always be located somewhere under `<path to your catkin_ws>/src`!__

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace       #initiates the catkin workspace
cd ~/catkin_ws
catkin_make                 #compiles the catkin workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Update your gazebo with the following commands

Add OSRF package repo

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

setup keys

    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

update gazebo

    sudo apt-get update
    sudo apt-get install gazebo7

### Mounting Hokuyo Laser Scanner onto the Turtlebot

We have modified the `turtlebot_description` package that has the simulated model of the turtlebot and put in some upgrades that should make your life easier. After following these
instructions, you should be able to visualise the laser ray projections in gazebo, and you will have a new stable topic that gets the laser data from the new scanner, which is `/laserscan`. We
have also enhanced the field of vision of the scan to 180 degrees and the maximum range to 30 m.

* Delete the existing `turtlebot_description` package using the following commands:
```
cd /opt/ros/kinetic/share
sudo rm -r turtlebot_description
```

* Take the modified `turtlebot_description` package from `mod_turtlebot_description` to your `Downloads`
folder. Extract the package from the compressed file.

* Move the extracted package from the `Downloads` folder to the location of the deleted package using the following commands:
```
cd ~/Downloads
sudo mv turtlebot_description /opt/ros/kinetic/share
```
If everything went alright, there should be a file named `hokuyo_urdf.xacro` at the location `/opt/ros/kinetic/share/turtlebot_description/urdf/sensors`

* Run the following commands to setup the new modified turtlebot as the default whenever you launch it in gazebo:
```
echo "export TURTLEBOT_3D_SENSOR=hokuyo" >> ~/.bashrc
source ~/.bashrc
```
Note that by doing this, the turtlebot will be launched with hokuyo laser scanner everytime you launch `turtlebot_gazebo`. If you want to go back to how everything was before you did all this, just delete
the `export TURTLEBOT_3D_SENSOR` line in the `.bashrc` file.

* Launch the `turtlebot_gazebo` package and hopefully, you should see your new turtlebot with enhanced superpowers like below and you should be able to see the scan data in the `/laserscan` topic. 
  
    To launch the simulation use: `roslaunch turtlebot_gazebo turtlebot_world.launch`

![](https://user-images.githubusercontent.com/7523395/35778702-3cfa014e-09c2-11e8-9f1d-bc1b28de14bc.png)

### Loading the Maze into the Simulation

All you have to do to load it is run the following command after launching the `turtlebot_gazebo package`:

`rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/<repo name>/maze_practice/model.sdf -sdf -model -maze -x 16 -y 5`

Be sure to enter your repository name correctly in the above command. An example:

`rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/ch_171744_tier4/maze_practice/model.sdf -sdf -model -maze -x 16 -y 5`

Your gazebo simulation should now look like below:

<img src="https://user-images.githubusercontent.com/7523395/35778687-047ef1e4-09c2-11e8-8766-cceff7a3e3f9.png" width="35%">

## How to use this ROS-Node (step-by-step)

1. Follow the install instructions
2. Download the repository in your catkin workspace
3. Run the simulation by using: `roslaunch turtlebot_gazebo turtlebot_world.launch`
4. Load the maze in your simulation: `rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/ch_171744_tier4/maze_practice/model.sdf -sdf -model -maze -x 16 -y 5`
    note: adopt the path to your system
5. Run the ROS-node to solve the maze by using: `roslaunch ch_171744_maze start_maze.launch`


## Description of the Solution

### Algorithm (pseudo code)

```
save the actual x- and y-position (loop-detection)
state = "WallDetection"

while not rospy.is_shutdown():
   if(state == "WallDetection")
        search for a wall to follow
        adjust the angle to next wall
        drive to the wall
        while(not wall_arrived):
            drive
        save the actual x- and y-position (loop-detection)
        make a turn 
        state = "wallFollow"

    elif(state == "wallFollow")
        if(loop_detected):
            state = "wallDetection"
        else:
            if(obstacles_detected)
                do turn ~90 degree
            else  
                follow wall using PID-controller
```

### Wall Detection

* In the image below you can see an example how the wall detection algorithm works. In this case the algorithm detects two walls. 
  Among these two walls, the wall with the biggest distance will be chosen. 

    ![wall1](https://user-images.githubusercontent.com/7523395/35778713-5676082a-09c2-11e8-834b-0c0dcef3948a.png)
    
    How does the wall detection work:
    
    There must be a predefined amount of points between the upper- and lower bound (see image below) . Another constraint is that the points must be next to each other. 
    Otherwise it will not be detected as a wall.  
    
    ![wall2](https://user-images.githubusercontent.com/7523395/35778718-66f8d4fc-09c2-11e8-9feb-6f9d673e93cf.png)

* Turn and move to the detected wall.

    ![wall3](https://user-images.githubusercontent.com/7523395/35778724-7468a978-09c2-11e8-9985-ed7814b221cc.png)

* After the robot arrived in front of the detected wall, save the actual x- and y-position (depicted as green circle in the image below) and do a turn. After that use the wall follow algorithm.

    ![wall4](https://user-images.githubusercontent.com/7523395/35778735-a46a11de-09c2-11e8-9727-44476faa12e4.png)

### Wall Follow

A PID-controller is used to adjust the direction of movement .

![wallf](https://user-images.githubusercontent.com/7523395/35778747-cd64ba1c-09c2-11e8-98cb-1f8ff4fdfebf.gif)


### Loop Detection

When the wall follow algorithm starts, the robot saves it actual position into a list of positions. This position will be used for loop detection.
If the robot hits any of these stored positions a loop is detected. In this case wall detection is used to reposition the robot.

![loopdetection](https://user-images.githubusercontent.com/7523395/35778761-ff2a2d0c-09c2-11e8-9a20-1cbcd6ce016a.gif)


## Author
Christian Högerle

## Licensing
The code in this project is licensed under MIT license.
