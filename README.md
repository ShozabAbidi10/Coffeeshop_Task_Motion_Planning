# AI for Robotics II course Assignment 2 (MSc Robotics Engineering, Unige)

This project is an extension of AI for robotics II (AIRO2) course’s assignment 1, in which we got familiar with the operational usage of an AI task planner and tested our understanding by developing a planning model for a toy robotic coffee shop. In this project we will use the same abstract scenario of the robotic coffee shop in order to get familiarized with the idea of modeling integrated task and motion planning problems in a belief space. 

The context of the assignment is the same, there is a waiter robot which is assigned with a task to serve the orders to the customers by visiting the coffee tables in an optimal way, the optimality is based on the distance travelled by the robot. Therefore, the robot has to reason on the discrete sequence of table visits to cover all the regions in the coffee shop. Unlike the previous assignment, we are considering that uncertainties will be involved with the robot’s motion, thus the robot has to navigate and localize itself while travelling from one region to another. Here ‘region’ refers to the table, in a realistic scenario a robot can be anywhere around the table while serving the customers but for simplicity we are considering that all the regions have a single way-point location associated with them in a 2D grid. It means that if the robot reaches the corresponding waypoint location of the region that region will be considered as being visited. The entire project can be found in this repository. Plesae find the intructios to run the project. 

## Project Installation:

To successfully run the project the first thing to do is to install the popf-tif planner, the details of which can be found in this repository: 
```
https://github.com/popftif/popf-tif
```
If you choose to use docker ubuntu image then keep in mind that the planner runs on the Ubuntu-18.04 Docker Official Image which doesn’t have any graphical interface. Therefore, to edit the planning files you need to mount a shared directory between your host system and docker image by using the following command:
```
docker run -dit -v path/in_your/host/folder:path/in/docker_container/folder --name your_name hypothe/ai4ro2_2
```
However the easiest way to run the project is to use this docker image which is developed by Marco Gabriele Fedozzi.
```
https://hub.docker.com/r/hypothe/ai4ro2_2
```
This docker image already contains all the basic setup to run the project. You just need to extract the visits_domain and visitis_module folders given along with this repository and replace them in ‘/root/ai4ro2’ directory. The visits_domain folder contains a PDDL domain, and problem files and the visits_module folder contains files to run the external planner. 

Besides this, make sure that you have installed an armadillo C++ library in your system which will be useful for complex matrix computation. 

In the ‘visits_domains’ folder you will also find a ‘region_poses’ file which contains the mapping from a region to its corresponding waypoint, waypoint.txt file which contains the geometric way-point locations for the four regions to visit as well as the starting region and the landmark.txt file which contains the landmarks to localize the robot. Make sure you have changed the directory addresses for waypoint.txt and landmark.txt files inside the ‘loadSolver’ function in the ‘VisitSolver.cpp’ file. The instructions to build the external module can be found in the ‘buildInstruction.txt’ file. Once built, the planner can be run using the command:
```
popf3-clp -x dom.pddl prob.pddl ../visits_module/build/libVisits.so region_poses
```
If the external module is built correctly, the planner should run without any errors and you the planner result on the terminal like this. 

![image](https://user-images.githubusercontent.com/61094879/130298228-e1e0d32a-8c56-4215-9ae9-1eff2e5adf00.png)







