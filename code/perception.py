import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

#Define a function to generate a mask to follow the wall
def mask_generator(mask,polygon_pts):
    cv2.fillPoly(mask,[polygon_pts],1)
    return mask


def ArcMaskGenerator(mask,axes,center,angle,startAngleDegrees,endAngleDegrees):
    cv2.ellipse(mask,center,axes,angle,startAngleDegrees,endAngleDegrees,1,thickness=cv2.FILLED)
    return mask

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

def find_rocks(img,mask,levels=(110,110,50)):
    rockpix = ((img[:,:,0]>levels[0])&(img[:,:,1]>levels[1])&(img[:,:,2]<levels[2]))
    rock_image = np.zeros_like(img[:,:,0])
    rock_image[rockpix] = 1
    return rock_image * mask


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))

    return warped, mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    Update_map = False
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    image = Rover.img
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])

    # 2) Apply perspective transform
    warped,mask = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    obs_map = np.absolute(np.float32(threshed)-1)*mask
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    
    Rover.vision_image[:,:,0] = obs_map * 255
    Rover.vision_image[:,:,2] = threshed * 255
    
    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)
    obs_xpix, obs_ypix = rover_coords(obs_map)
    # 6) Convert rover-centric pixel values to world coordinates
    world_map_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    x_pix_world, y_pix_world = pix_to_world(xpix, ypix, Rover.pos[0],Rover.pos[1],Rover.yaw, world_map_size,scale)
    obs_x_pix_world, obs_y_pix_world = pix_to_world(obs_xpix, obs_ypix, Rover.pos[0],Rover.pos[1],Rover.yaw, world_map_size,scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    update_map_threshold = 0.5
    if Rover.pitch > 180 and Rover.roll > 180:
        if (360-Rover.pitch) < update_map_threshold and (360-Rover.roll) < update_map_threshold:
            Update_map = True
    elif Rover.pitch < 180 and Rover.roll > 180:
        if (Rover.pitch) < update_map_threshold and (360-Rover.roll) < update_map_threshold:
            Update_map = True
    elif Rover.pitch > 180 and Rover.roll < 180:
        if (360-Rover.pitch) < update_map_threshold and (Rover.roll) < update_map_threshold:
            Update_map = True
    else:
        if (Rover.pitch) < update_map_threshold and (Rover.roll) < update_map_threshold:
            Update_map = True
    
    #print("Rover Pitch: ",Rover.pitch,"; Rover Roll: ",Rover.roll, "; Update map: ",Update_map)
    if Update_map:
        Rover.worldmap[obs_y_pix_world, obs_x_pix_world,0] += 1
        Rover.worldmap[y_pix_world, x_pix_world, 2] += 20
    
    #polygon_pts = np.array([[int(image.shape[1]/2-1),image.shape[0]-1],
    #                        [int(image.shape[1]*0.45),0],
    #                        [int(image.shape[1]*0.66),0],
    #                        [int(image.shape[1]/2+1),image.shape[0]-1]],
    #                        np.int32)

    #polygon_pts = np.array([[150,60],
    #                        [240,100]
    #                        [165,155],
    #                        [150,155]],
    #                        np.int32)

    cen = (160,160)

    RockMask = np.zeros_like(image[:,:,0])
    RockMask = ArcMaskGenerator(RockMask, (45,70),cen,-144,0,80)
    #RockMask[int(RockMask.shape[0]*0.90):,:] = 0
    rock_map = find_rocks(warped,RockMask,levels=(110,110,50))
    #Rover.vision_image[:,:,0] = RockMask * 255

    #polygon_pts = np.array([[140,0],
    #                        [288,64],
    ##                        [160,160]],
      #                      np.int32)
    
    
    WallFollowMask = np.zeros_like(image[:,:,0])
    #WallFollowMask = ArcMaskGenerator(WallFollowMask,160,(160,160),-30,-80)
    #WallFollowMask = ArcMaskGenerator(WallFollowMask,50,(160,160),-60,-120)
    #WallFollowMask = ArcMaskGenerator(WallFollowMask,30,(160,160),-120,-150)
    #WORKING well: went inside the rocks. problems with sample
    #WallFollowMask = ArcMaskGenerator(WallFollowMask,(140,25),cen,-144,0,95) 

    #Optimal working no issues found until now
    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    found_direction = False
    min_collision = 0
    min_collision_i = -145
    too_much_collision = 3

    for i in range(-108,-72,2):
        FrontalCollisionMask = np.zeros_like(image[:,:,0])
        FrontalCollisionMask = ArcMaskGenerator(FrontalCollisionMask,(20,3),cen,i,-90,90) 
        FrontalCollisionMask[int(FrontalCollisionMask.shape[0]*0.95):,:] = 0
        x_collision, _ = rover_coords(abs(1-threshed) * FrontalCollisionMask)
        if len(x_collision) == 0:
            found_direction = True
            break

        if (min_collision == 0 or len(x_collision) < min_collision):
            min_collision = len(x_collision)
            min_collision_i = i

    
    if found_direction:
        Rover.drive_direction = i
        Rover.found_direction = found_direction
    else:
        if (min_collision == 0 or min_collision > too_much_collision):
            Rover.drive_direction = 0
            Rover.found_direction = False
        else:
            Rover.drive_direction = min_collision_i
            Rover.found_direction = True
    

    #Rover.vision_image[:,:,2] = FrontalCollisionMask * 255

    WallFollowMask = ArcMaskGenerator(WallFollowMask,(65,23),cen,Rover.drive_direction,-90,83) 
    
    
    #WallFollowMask = ArcMaskGenerator(WallFollowMask,(140,60),cen,-144,0,90)
    #WallFollowMask = ArcMaskGenerator(WallFollowMask,50,(160,160),-60,-120)
    #WallFollowMask = ArcMaskGenerator(WallFollowMask,30,(160,160),-30,-60)

    #WallFollowMask[:,int(WallFollowMask.shape[1]*0.72):] = 0
    #WallFollowMask[int(WallFollowMask.shape[0]*0.96):,:] = 0
    #Rover.vision_image[:,:,2] = WallFollowMask * 255

    xpix_nav,ypix_nav = rover_coords(threshed * WallFollowMask)
    #xpix_nav,ypix_nav = rover_coords(threshed)
    #xpix_one_side, ypix_one_side = rover_coords(threshed * WallFollowMask)
    #xpix_nav = np.append(xpix_nav,xpix_one_side)
    #xpix_nav = xpix_one_side
    #ypix_nav = np.append(ypix_nav,ypix_one_side)
    #ypix_nav = ypix_one_side


    

    if rock_map.any():
        rock_x,rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x,rock_y, Rover.pos[0],Rover.pos[1],Rover.yaw, world_map_size,scale)
        dist_rock, angle_rock = to_polar_coords(rock_x,rock_y)
        Rover.nav_dists = dist_rock
        Rover.nav_angles = angle_rock

        rck_idx = np.argmin(dist_rock)
        rck_xcen = rock_x_world[rck_idx]
        rck_ycen = rock_y_world[rck_idx]
        Rover.worldmap[rck_ycen, rck_xcen, :] = 255
        Rover.vision_image[:,:,1] = rock_map * 255
        Rover.Rock_found = True
    
    #if not Rover.mode == 'RockPickUP':
    else:
        Rover.vision_image[:,:,1] = 0

        #If no rocks found move in freespace
        distance, angles = to_polar_coords(xpix_nav,ypix_nav)
        Rover.nav_dists = distance
        Rover.nav_angles = angles
        Rover.Rock_found = False
    
    
    
    
        
        
        

    #print('Navigation angles: ', np.mean(Rover.nav_angles))
    #print('Navigation Distance: ',np.mean(Rover.nav_dists))
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles


    
 
    
    
    return Rover
