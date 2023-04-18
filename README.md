# TIpsyTurtle

## New Environment Setup
* The Turtlebot4 has a RPi and a Create3 module that both have to connect to wifi and eachother
* Follow https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html
* If you have any networking issues contact Lawson Hutter or Kevin McGrath
* Follow https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html
* Save the new map and SCP it to the Turtlebot4
* Echo the clicked point topic in a new terminal
* Publish the fridge and couch position and add them to the turtle_controller.py project

## How to run:
* Start TurtleBot4
* Wait for it to connect both the RPi and Create3 (Will make a happy sound)
* Open 2 SSH terminals to the RPi
* On the first one:
** ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=<YourMap>.yaml
* On the Second One:
** cd turtlebot4_ws
** source install/setup.bash
