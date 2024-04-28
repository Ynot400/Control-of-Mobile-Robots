# Control-of-Mobile-Robots

Using a virtual robotics simulation called Webots, this repository initializes a four wheeled simulated robot called RosBot to do assigned "labs" that go over **Kinematics, PID controllers, Wall Following, Motion Planning with Bug Zero Algorithm, and Localization.** 

The repository expands on the **FAIRIS-Lite** project framework, which can be accessed through this [link](https://github.com/biorobaw/FAIRIS-Lite). The framework allows for users to integrate control logic for navigation of the RosBot within the Webots simulator. 

My additions to this framework involve programming the RosBot's **["controllers,"](WebotsSim/controllers)** which are used to control the actions of the simulated robot. The core features inside these "controllers" are control functions that were coded in **[MyRobot.py](WebotsSim/libraries/MyRobot.py)**, which is an extension that builds upon the functionalities of the RosBot class. 

In each of my controller folders, you will find the lab assignment, which explains the task(s) for the program, as well as a lab report, which will showcase the mathematical computations for the program as well as an overall conclussion of what went about when finishing the lab.

