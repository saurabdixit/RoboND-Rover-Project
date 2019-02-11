import numpy as np
import time

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function



def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    #Take a step back and move forward again if robot get stuck

    
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.samples_located > 5:
        Rover.mode = 'stop' 
    if Rover.nav_angles is not None:

        if np.array_equal(np.round(Rover.old_pos,decimals=1),np.round(Rover.pos,decimals=1)) and (not Rover.picking_up) and (time.time()-Rover.time_spent_by_rover_on_one_location > 8):
            Rover.mode = 'stucked'
        # Check for Rover.mode status
        if Rover.mode == 'stucked':
            Rover.Rock_found = False
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = -15 # Could be more clever here about which way to turn
            
            if time.time()-Rover.time_spent_by_rover_on_one_location > 2:
                Rover.mode = 'forward'

        elif Rover.Rock_found:
            if Rover.vel > 0.7:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
            else:
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                if not Rover.near_sample:
                    if Rover.vel < 0.5:
                        Rover.throttle = Rover.throttle_set  
                        Rover.brake = 0  
                    else:
                        Rover.throttle = 0
                elif Rover.near_sample:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.mode = 'stop'
        
        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            #stop_condition = False
            #Rover.max_vel = 2.0
            #stop_condition = len(Rover.nav_angles) < Rover.stop_forward
            
            #if not stop_condition and (Rover.collision_angle is not None and len(Rover.collision_angle) > 0):
            #    Rover.collision_direction = np.clip(np.mean(Rover.collision_angle * 180/np.pi), -15, 15)
                #stop_condition = abs(Rover.collision_direction - Rover.steer) < 0.5
            #    stop_condition = True
            #    if stop_condition:
            #        Rover.max_vel = 0.5
            
            #print("Stop condition:",stop_condition)
                
            if Rover.found_direction:
            #    Rover.steer = np.clip(-1*Rover.drive_direction - 90, -15, 15)
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                Rover.brake = 0
                if Rover.vel < Rover.max_vel:
                    Rover.throttle = Rover.throttle_set
                else:
                    Rover.throttle = 0
            else:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.mode = 'stop'
            

            #if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
            #    if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
            #        Rover.throttle = Rover.throttle_set
            #    else: # Else coast
            #        Rover.throttle = 0
            #    Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)


                #My stop condition
            #    if stop_condition:
                    
            #        if Rover.vel > Rover.max_vel:
            #            Rover.brake = 0.05
            #        else:
            #            Rover.brake = 0.0
            #        if Rover.vel == 0:
            #            Rover.mode = 'stop'
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            #elif stop_condition:
            #        # Set mode to "stop" and hit the brakes!
            #        Rover.throttle = 0
            #        # Set brake to stored brake value
            #        Rover.brake = Rover.brake_set
            #        Rover.steer = 0
            #        Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    if Rover.samples_located < 5:
                        Rover.mode = 'forward'
        
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

   
    return Rover

