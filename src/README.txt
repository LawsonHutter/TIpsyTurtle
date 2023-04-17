Notes about this process:

package.xml : contains dependencies

CREATING A NODE:
    In the node file, the file with the same nam as the package,
    touch my_first_node.py
    chmod +x my_first_node.py

CREATE EXECUTABLE:
    In setup.py
    In console_scripts
    Add quotes " "
    ex)
        "test_node = tipsy_turtle.my_first_node:main"
    Colcon build after this from workspace

RUNNING EXECUTABLE
    source ~/.bashrc
    ros2 run tipsy_turtle test_node

SKIP BUILD WHEN UPDATING CODE
    colcon build --symlink-install
    source ~/.bashrc

    Now when you save your python it automatically builds
    All you have to do is run it

COMMUNICATION
    Nodes publish topics
    Other Nodes subscribe to those topics

    To understand the nodes and data between nodes
    ros2 topic info /topic
    ros2 node info /nodet

    Lists data on topic
    ros2 topic echo /topic

SEE MORE ABOUT A TOPIC 
    ros2 topic info /topic
        retuns a Type 
    
    ros2 interface show type 
        ellaborates on data type 

SERVICES
    ros2 interface show service 
    Allows you to make requests and returns something

LAUNCH FILE 
    Start multiple nodes at once

git config user.email "lawsonhutter@ufl.edu"
git config user.name "lawsonhutter"

GET MAP OFF TURTLE 
    scp ubuntu@192.168.1.26:/lhutt_apt.yaml /home/syqua/Desktop/ros2_ws/src/tipsy_turtle/maps

    scp k_mcg_apt.yaml ubuntu@192.168.0.234:~/turtlebot4_ws/src/my_nav/maps