# youbot_gui
System requirements:
- ROS Kinetic
- Ubuntu 16.04 LTS
- MoveIt!
- OpenCV

Installation:

Create and initialize catkin workspace (i. e. youbot_gui):

    mkdir -p ~/youbot_gui/src
    cd ~/youbot_gui/src
    catkin_init_workspace
    cd ..
    catkin_make
    
Clone the youBot GUI sources to /src folder:
    
    cd ~/youbot_gui/src
    git clone https://github.com/arek-szczech/youbot.git

Add the repository folder to the ROS_PACKAGE_PATH environment variable.

Compile the youbot GUI by typing:

    cd ~/youbot_gui
    catkin_make
    
Usage:

    cd ~/youbot_gui
    source devel/setup.bash
