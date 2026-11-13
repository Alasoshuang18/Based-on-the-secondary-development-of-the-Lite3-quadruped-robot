"""Keep command headroom below the unchanged 0.20 measured-motion audit."""
import copy


def with_tracking_headroom(parameters):
    parameters=copy.deepcopy(parameters)
    controller=parameters['controller_server']['ros__parameters']['FollowPath']
    smoother=parameters['velocity_smoother']['ros__parameters']
    controller['desired_linear_vel']=.10
    controller['rotate_to_heading_angular_vel']=.10
    controller['regulated_linear_scaling_min_radius']=1.
    smoother['max_velocity']=[.10,0.,.10]
    smoother['min_velocity']=[0.,0.,-.10]
    # Preserve commanded curvature when the yaw cap scales a velocity command.
    smoother['scale_velocities']=True
    return parameters
