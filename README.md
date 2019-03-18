[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Search and Sample Return Project
This project helped us in understanding how the perception, decision making and actuation are handeled in real life robotics.

[![Rover in action](http://img.youtube.com/vi/Zu-8fPl4TfM/0.jpg)](http://www.youtube.com/watch?v=Zu-8fPl4TfM)

## Notebook Analysis 
### Rock Identification
I have added a function named find_rocks. This function identifies the golden sample present in the image taken by the Rover.
```python
def find_rocks(img,levels=(110,110,50)):
    rockpix = ((img[:,:,0]>levels[0])&(img[:,:,1]>levels[1])&(img[:,:,2]<levels[2]))
    rock_image = np.zeros_like(img[:,:,0])
    rock_image[rockpix] = 1
    return rock_image
```
Following is the example of input amd output image.
![](./misc/find_rock.png?raw=true "Find Rock function input and output")
### Obstacle Identification
To identify the obstacles in the environment, I have identified the areas which are not navigable.
```python
threshold = color_thresh(warped)                    #Navigable areas
obs_map = np.absolute(np.float32(threshold)-1)      #Obstacles
```
There was a mask introduced in the perspective_transform function to make sure that robot captures just the relevant part of the image. Hence, the perspective transform function was changed to 
```python
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    #Added following line
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    
    return warped,mask
```
After multiplying the mask with the threshold image, we got following results:
![](./misc/warped.png?raw=true "Warped")

### process_image() function
I modified the process image function to the following:
```python
def process_image(img):
    # Example of how to use the Databucket() object defined above
    # to print the current x, y and yaw values 
    # print(data.xpos[data.count], data.ypos[data.count], data.yaw[data.count])

    image = np.copy(img)
    # TODO: 
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshold = color_thresh(warped)
    obs_map = np.absolute(np.float32(threshold)-1) * mask
    
    # 4) Convert thresholded image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshold)
    obs_xpix, obs_ypix = rover_coords(obs_map)
    # 5) Convert rover-centric pixel values to world coords
    x_pix_world, y_pix_world = pix_to_world(xpix, ypix, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], 200,10)
    obs_x_pix_world, obs_y_pix_world = pix_to_world(obs_xpix, obs_ypix, data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], 200,10)
    # 6) Update worldmap (to be displayed on right side of screen)
        # Example: data.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          data.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          data.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    data.worldmap[y_pix_world, x_pix_world, 2] = 255
    data.worldmap[obs_y_pix_world, obs_x_pix_world, 0] = 255
    navpix = data.worldmap[:,:,2]>0
    data.worldmap[navpix,0] =0
    
    rock_map = find_rocks(warped,levels=(110,110,50))
    if rock_map.any():
        rock_x,rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x,rock_y,data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], 200,10)
        
        data.worldmap[rock_y_world,rock_x_world,:] = 255
        
    
    
    
    # 7) Make a mosaic image, below is some example code
        # First create a blank image (can be whatever shape you like)
    output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
        # Next you can populate regions of the image with various output
        # Here I'm putting the original image in the upper left hand corner
    output_image[0:img.shape[0], 0:img.shape[1]] = img

        # Let's create more images to add to the mosaic, first a warped image
    warped,mask = perspect_transform(img, source, destination)
        # Add the warped image in the upper right hand corner
    output_image[0:img.shape[0], img.shape[1]:] = warped

        # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
        # Flip map overlay so y-axis points upward and add to output_image 
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)


        # Then putting some text over the image
    cv2.putText(output_image,"Populate this image with your analyses to make a video!", (20, 20), 
                cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    if data.count < len(data.images) - 1:
        data.count += 1 # Keep track of the index in the Databucket()
    
    return output_image
```
1) Initially, I defined the grid size and bottom offset from the rover to the camera image start value.
2) Then, I used perspective_transform function to identify warped image and got the mask as well.
3) The perspective transform function takes three inputs: image, source, and destination. The image is captured from the data. However, the source was marked manually using the grid plot in simulation. The destination was calculated based on grid size.
4) After that I appled color_thresh functions to identify navigable terrain and obstacle map
5) Then, Identified the pixel values of the where navigable terrain using the images generated from color_thresh function. Similarly, identifed the pixel values of obstacles
6) I converted the pixel values of navigable terrain and obstacle map to the world coordinates so that we can plot it on map.
7) On the map, when robot moves, it marks navigable terrain in blue and obstacles as red.
8) The next step was to find the rock in the give scene. I used the find_rocks function on the warped image generated from perspective_transform function to find the pixel location of the rock. Similar to point 6, I converted the pixel values of rock to world frame of reference. The rock will be marked as white dot on the map.



## Autonomous Navigation and Mapping


### Optimal Implementation Theory
There are number of ways to accomplish this task. However, the best way  is to use Depth first search. Here is how depth first search will work on this problem: Suppose the robot is at the center of the map. There are three possible direction that robot can go into. Let's call that three branches. It will completely visit one branch or let's say map one branch before visiting another one. This will ensure that no brances are visited twice. In this case, the limitation we have is that the sensor that we are using is not scanning complete 360 degree feild of view. Hence, to implment depth first search, we would have to rotate the robot at current position, identify all the navigable nodes and mark it. Ofcourse, we would have to divide the scanned area in definite sized grid otherwise we will end up with infinite amount of nodes and robot will have to visit all of them.

### My Implementation
As we have limitation over the sensor, I went with the approach of following the wall as suggested in hints. To accomplish this task, I used a mask which only looks at the approximately center part to right side of the navigable terrain and calculates the steering angle based on that. Hence, my robot always follow the right wall.

### Rock Identification and pick up
My robot behaves in greedy manner. Basically, as soon as the robot identifies the golden samples, it picks it up. However, the policy for searching is following the right wall. Thus, I was running into issues of skipping the rest of the path whenever the robot identifies a rock on the other i.e. left side of the pathway. Hence, I have applied the same mask i.e. don't detect the rock which are on the left side of robot and keep going. The robot will find it on its way back.

### Issue with current implementation
There is one issue in my implementation and I know how to solve it. However, I am getting too late for submitting this project and would like to submit this project soon.

The robot sometimes run into obstacles as it is not utilizing the full field of view to plan the path. It makes it stuck sometimes. To make it unstuck, I have added the logic which makes the robot turn on its own position if it get stuck at one position for more than 8 secs. One more thing that I wanted to add is to define the configuration space (C-Space) for the robot. So that robot would avoid going close to the obstacles and plan path away from the obstacles.

### Remaining Implementation:
The only thing that is remaining in this implementation is to command the robot to go to the start position when all the samples are collected. The algorithm that could do this quickly is RRT* as we know the start position of the robot and we know the current position of the robot. We can start the Rapidly-growing random tree on the start position and on the current position and wait for them to merge. Once they merge, we will have path to follow. We can also make it goal directed so that the tree can grow in the direction of where it wants to go.



