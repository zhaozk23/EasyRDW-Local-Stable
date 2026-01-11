from utils.constants import *
from utils.space import *
import math

# TODO: Implement your own logic in the following functions.
def calc_gain(user : UserInfo, physical_space : Space, delta : float):
    """
    Return three gains and the direction (+1 or -1) when cur_gain used. Implement your own logic here.
    All lengths are in meters(m).
    user.v : m/frame.
    user.w : rad/frame.
    delta is the frame time interval in seconds. You might need it at some point.
    """
    # # S2C
    # center_x, center_y = physical_space.get_center()
    # angle = math.atan2(center_y - user.y, center_x - user.x)
    # angle_diff = angle - user.angle
    # angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi # [-pi, pi]
    # # judge whether change target or not
    # if abs(angle_diff)>math.pi*8/9:
    #     # generate new target 90 degrees from the current user angle and 4 meters away from user, with the direction close to center
    #     target_x = user.x + 100 * math.cos(user.angle + math.pi/2)
    #     target_y = user.y + 100 * math.sin(user.angle + math.pi/2)
    #     # judge whether the target makes the user closer to the center  
    #     if (target_x - user.x) * (-user.x + center_x) + (target_y - user.y) * (-user.y + center_y) < 0:
    #         target_x = user.x + 100 * math.cos(user.angle - math.pi/2)
    #         target_y = user.y + 100 * math.sin(user.angle - math.pi/2)
    # else:
    #     target_x = center_x
    #     target_y = center_y
        
    # dist = math.sqrt((target_x - user.x) ** 2 + (target_y - user.y) ** 2)
    # direction = 1 if angle_diff > 0 else -1
    # curvature_gain_radius = MIN_CUR_GAIN_R
   
    # rotation_gain = MAX_ROT_GAIN
    # if user.w * direction < 0:
    #     rotation_gain = MIN_ROT_GAIN
    # # calculate the rotation caused by curvature

    # if dist < 125:
    #     curvature_gain_radius = MIN_CUR_GAIN_R * (2.5/(dist+1.25))
    # return MAX_TRANS_GAIN, rotation_gain, curvature_gain_radius, direction
    center_x, center_y = physical_space.get_center()
    radius = 500
    targets = [
        (center_x + radius, center_y),  # Target 1
        (center_x - radius / 2, center_y + math.sqrt(3) * radius / 2),  # Target 2
        (center_x - radius / 2, center_y - math.sqrt(3) * radius / 2),  # Target 3
    ]
    # find the target closest to being in front of the user
    closest_target = None
    smallest_bearing_difference = math.pi
    bearing_diff = None
    for target_x, target_y in targets:
        # Compute the angle to the target
        to_target_x = target_x - user.x
        to_target_y = target_y - user.y
        angle_to_target = math.atan2(to_target_y, to_target_x)

        # Calculate bearing difference to the user's current heading
        bearing_diff = angle_to_target - user.angle
        bearing_diff = math.atan2(math.sin(bearing_diff), math.cos(bearing_diff))  # Normalize to [-π, π]

        if abs(bearing_diff) < smallest_bearing_difference:
            smallest_bearing_difference = abs(bearing_diff)
            closest_target = (target_x, target_y)
    direction = 1 if bearing_diff > 0 else -1
    curvature_gain_radius = MIN_CUR_GAIN_R
    translation_gain = MAX_TRANS_GAIN
    rotation_gain = MAX_ROT_GAIN
    if user.w*direction < 0:
        rotation_gain = MIN_ROT_GAIN
    dist = math.sqrt((closest_target[0] - user.x) ** 2 + (closest_target[1] - user.y) ** 2)
    if dist < 125:
        curvature_gain_radius = MIN_CUR_GAIN_R*(2.5/(dist+1.25))
        
    return translation_gain, rotation_gain, curvature_gain_radius, direction 

def update_reset(user : UserInfo, physical_space : Space, delta : float):
    """
    Return new UserInfo when RESET. Implement your RESET logic here.
    """
    center_x, center_y = physical_space.get_center()
    user.x = center_x
    user.y = center_y
    user.angle = (user.angle + math.pi) % (2 * math.pi)
    return user
