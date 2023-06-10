# hands-on-planning-project-group-7
**This project belongs to the Universitat de Girona. It is forbidden to publish this project or any derivative work in any public repository.**

## Group Members:

This project has been carried out by:

* Asraa Al-Ali
* Mohamed Alsalami
* Amna AlMehrzi


## How to use it:

After adding the package to your workspace, access the terminal and execute the launch file named *octomap.launch* by running the following command:

```
roslaunch turtlebot_simulation octomap.launch 
```
Two windows should appear: a Stonefish simulation window and an RViz visualization window. 
After that open new terminal to start the velocity node *twist_vel.py* to allow converting it from twist message to the TurtleBotby running the following command:

```
rosrun turtlebot_simulation twist_vel.py 
```

Next, open a third terminal tab and run the node named *turtlebot_online_path_planning* by running the following command:

```
rosrun turtlebot_simulation turtlebot_online_path_planning_node.py
```

The exploration will start automatically, choosing a new goal point and computing a path to it.
