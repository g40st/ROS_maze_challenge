# maze solver (ROS)




## Installing / Getting started (Linux)

The turtlebot that you worked with until now did not really have a laser scanner. It had a camera sensor and the sensor data from the camera was being converted to laser data, which is one of the
reasons why there were issues with the `/scan` topic whenever gazebo was reset.

Therefore, we have modified the `turtlebot_description` package that has the simulated model of the turtlebot and put in some upgrades that should make your life easier. After following these
instructions, you should be able to visualise the laser ray projections in gazebo, and you will have a new stable topic that gets the laser data from the new scanner, which is `/laserscan`. We
have also enhanced the field of vision of the scan to 180 degrees and the maximum range to 30 m.

Consider this our Christmas present to you! ;)

* Delete the existing `turtlebot_description` package using the following commands:
```
cd /opt/ros/kinetic/share
sudo rm -r turtlebot_description
```

* Download the modified `turtlebot_description` package from this [link] (https://fbe-gitlab.hs-weingarten.de/mat-iki/amr-mat/blob/master/turtlebot_hokuyo/turtlebot_description.tar.gz) to your `Downloads`
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


![](https://fbe-gitlab.hs-weingarten.de/mat-iki/amr-mat/raw/master/.img/turtlebot_hokuyo.png)


## Author
Christian Högerle

## Licensing
The code in this project is licensed under MIT license.
