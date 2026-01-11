from utils.constants import *
from utils.space import *
import math

# use the translated visibility-polygon RDW implementation
from vis_poly_rdw.vec2 import Vec2
from vis_poly_rdw.vis_poly_rdw import VisPolyRdw

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
    # Build environment dicts expected by VisPolyRdw
    phys_vertices = [ (t[0], t[1]) for t in physical_space.border ]
    phys_obs = [ [(p[0], p[1]) for p in obs] for obs in physical_space.obstacle_list ]

    # convert to Vec2 objects for pasumi_py
    env_phys = {
        'vertices': [Vec2(x,y) for (x,y) in phys_vertices],
        'obstacles': [[Vec2(x,y) for (x,y) in obs] for obs in phys_obs]
    }
    # virtual environment: same boundary but no obstacles (simple assumption)
    env_virt = {
        'vertices': [Vec2(x,y) for (x,y) in phys_vertices],
        'obstacles': []
    }

    vis = VisPolyRdw()
    pos = Vec2(user.x, user.y)
    heading = user.angle

    phys_poly = vis.get_vis_poly(pos, env_phys, heading)
    virt_poly = vis.get_vis_poly(pos, env_virt, heading)

    # choose slice matching logic (closest area)
    if not phys_poly.slices or not virt_poly.slices:
        # fallback conservative gains
        return MAX_TRANS_GAIN, MAX_ROT_GAIN, MIN_CUR_GAIN_R, 1

    virt_slice = virt_poly.slices[0]
    best_slice = None
    best_diff = float('inf')
    for s in phys_poly.slices:
        if abs(s.theta_offset) > math.pi * 0.5:
            continue
        diff = abs(virt_slice.area - s.area)
        if diff < best_diff:
            best_diff = diff
            best_slice = s
    if best_slice is None:
        best_slice = phys_poly.slices[0]

    # steer target (direction vector)
    steer_target = Vec2(user.x, user.y) + best_slice.bisector

    # direction to steer: sign of angle from user heading to steer target
    user_dir = Vec2(math.cos(user.angle), math.sin(user.angle))
    to_target = (steer_target - Vec2(user.x, user.y))
    td_len = math.hypot(to_target.x, to_target.y)
    if td_len == 0:
        direction = 1
    else:
        to_target_n = Vec2(to_target.x / td_len, to_target.y / td_len)
        angle_to_gradient = math.atan2(user_dir.x * to_target_n.y - user_dir.y * to_target_n.x,
                                       user_dir.x * to_target_n.x + user_dir.y * to_target_n.y)
        direction = 1 if angle_to_gradient > 0 else -1

    # translation gain: ratio of available space in phys/virt (avg height)
    virt_h = virt_slice.avg_height if virt_slice.avg_height > 0 else 1.0
    phys_h = best_slice.avg_height if best_slice.avg_height > 0 else virt_h
    trans_gain = phys_h / virt_h
    trans_gain = max(MIN_TRANS_GAIN, min(MAX_TRANS_GAIN, trans_gain))

    # rotation gain: prefer large gain when user's angular velocity aligns with desired turn
    rota_gain = MAX_ROT_GAIN
    if abs(user.w) > 1e-6:
        # if user is turning opposite to desired direction, reduce gain
        if user.w * direction < 0:
            rota_gain = MIN_ROT_GAIN
        else:
            rota_gain = MAX_ROT_GAIN

    # curvature radius: reuse existing heuristic from controller
    dist = math.hypot(steer_target.x - user.x, steer_target.y - user.y)
    cur_r = MIN_CUR_GAIN_R
    if dist < 125:
        cur_r = MIN_CUR_GAIN_R * (2.5 / (dist + 1.25))

    return trans_gain, rota_gain, cur_r, direction

def update_reset(user : UserInfo, physical_space : Space, delta : float):
    """
    Return new UserInfo when RESET. Implement your RESET logic here.
    """
    center_x, center_y = physical_space.get_center()
    user.x = center_x
    user.y = center_y
    user.angle = (user.angle + math.pi) % (2 * math.pi)
    return user
