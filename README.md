[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Search and Sample Return Project


![alt text][image_0] 

This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) and it will give you first hand experience with the three essential elements of robotics, which are perception, decision making and actuation.  You will carry out this project in a simulator environment built with the Unity game engine.  

## Optimal Implementation Theory
There are number of ways to accomplish this task. However, the best way  is to use Depth first search. Here is how depth first search will work on this problem: Suppose the robot is at the center of the map. There are three possible direction that robot can go into. Let's call that three branches. It will completely visit one branch or let's say map one branch before visiting another one. This will ensure that no brances are visited twice. In this case, the limitation we have is that the sensor that we are using is not scanning complete 360 degree feild of view. Hence, to implment depth first search, we would have to rotate the robot at current position, identify all the navigable nodes and mark it. Ofcourse, we would have to divide the scanned area in definite sized grid otherwise we will end up with infinite amount of nodes and robot will have to visit all of them.

## My Implementation
As we have limitation over the sensor, I went with the approach of following the wall as suggested in hints. To accomplish this task, I used a mask which only looks at the approximately center part to right side of the navigable terrain and calculates the steering angle based on that. Hence, my robot always follow the right wall.

## Rock Identification and pick up
My robot behaves in greedy manner. Basically, as soon as the robot identifies the golden samples, it picks it up. However, the policy for searching is following the wall. Thus, I was running into issues of skipping the rest of the path whenever the robot identifies a rock on the other left side of the pathway. Hence, I have applied the same mask i.e. don't detect the rock which are on the left side of robot and keep going. The robot will find it on its way back.

## Issue with current implementation
There is one issue in my implementation and I know how to solve it. However, I am getting too late for submitting this project and would like to submit this project soon.

The robot sometimes run into obstacles as it is not utilizing the full field of view to plan the path. It makes it stuck sometimes. To make it unstuck, I have added the logic which makes the robot turn on its own position if it get stuck at one position for more than 8 secs. One more thing I wanted to add is to define the configuration space (C-Space) for the robot. So that robot would consider that and plan away from the obstacles

## Remaining Implementation:
The only thing that is remaining in this implementation is to command the robot to go to the start position when all the samples are collected. The algorithm that could do this quickly is RRT* as we know the start position of the robot and we know the current position of the robot. We can start the Rapidly-growing random tree on the start position and on the current position and wait for them to merge. Once they merge, we will have path to follow. We can also make it goal directed so that the tree can grow in the direction of where it wants to go.



