import geopandas as gpd
from matplotlib.axes._axes import Axes
from uav_v2 import UAV_v2
import numpy as np

def compute_time_to_impact(host_uav:UAV_v2, other_uav:UAV_v2):
    host_pos = host_uav.current_position
    other_pos = other_uav.current_position

    host_vel = np.array([host_uav.current_speed*np.cos(host_uav.current_heading), host_uav.current_speed*np.sin(host_uav.current_heading)])
    other_vel = np.array([other_uav.current_speed*np.cos(other_uav.current_heading), other_uav.current_speed*np.sin(other_uav.current_heading)])

    combined_radius = host_uav.radius + other_uav.radius

    v_rel = host_vel - other_vel

    coll_cone_vec1, coll_cone_vec2 = tangent_vecs_from_external_pt(host_uav.current_position.x, 
                                                                   host_uav.current_position.y, 
                                                                   other_uav.current_position.x, 
                                                                   other_uav.current_position.y, 
                                                                   combined_radius)

    if coll_cone_vec1 is None:
        # collision already occurred ==> collision cone isn't meaningful anymore
        return 0.0
    else: 
        # check if v_rel btwn coll_cone_vecs
        # (B btwn A, C): https://stackoverflow.com/questions/13640931/how-to-determine-if-a-vector-is-between-two-other-vectors)

        if (np.cross(coll_cone_vec1, v_rel) * np.cross(coll_cone_vec1, coll_cone_vec2) >= 0 and 
            np.cross(coll_cone_vec2, v_rel) * np.cross(coll_cone_vec2, coll_cone_vec1) >= 0):
            # quadratic eqn for soln to line from host agent pos along v_rel vector to collision circle
            # circle: (x-a)**2 + (y-b)**2 = r**2
            # line: y = v1/v0 *(x-px) + py
            # solve for x: (x-a)**2 + ((v1/v0)*(x-px)+py-a)**2 = r**2
            v0, v1 = v_rel
            if abs(v0) < 1e-5 and abs(v1) < 1e-5:
                # agents aren't moving toward each other ==> inf TTC
                return np.inf

            px, py = host_pos
            a, b = other_pos
            r = combined_radius
            if abs(v0) < 1e-5: # vertical v_rel (solve for y, x known)
                print("[warning] v0=0, and not yet handled")
                x1 = x2 = px
                A = 1
                B = -2*b
                C = b**2+(px-a)**2-r**2
                y1 = (-B + np.sqrt(B**2 - 4*A*C)) / (2*A)
                y2 = (-B - np.sqrt(B**2 - 4*A*C)) / (2*A)
            else: # non-vertical v_rel (solve for x)
                A = 1+(v1/v0)**2
                B = -2*a + 2*(v1/v0)*(py-b-(v1/v0)*px)
                C = a**2 - r**2 + ((v1/v0)*px - (py-b))**2

                det = B**2 - 4*A*C
                if det == 0:
                    print("[warning] det == 0, so only one tangent pt")
                elif det < 0:
                    print("[warning] det < 0, so no tangent pts...")

                x1 = (-B + np.sqrt(B**2 - 4*A*C)) / (2*A)
                x2 = (-B - np.sqrt(B**2 - 4*A*C)) / (2*A)
                y1 = (v1/v0)*(x1-px) + py
                y2 = (v1/v0)*(x2-px) + py

            d1 = np.linalg.norm([x1-px, y1-py])
            d2 = np.linalg.norm([x2-px, y2-py])
            d = min(d1, d2)
            spd = np.linalg.norm(v_rel)
            return d / spd 
        else:
            return np.inf

def tangent_vecs_from_external_pt(xp, yp, a, b, r):
    # http://www.ambrsoft.com/TrigoCalc/Circles2/CirclePoint/CirclePointDistance.htm
    # (xp, yp) is coords of pt outside of circle
    # (x-a)**2 + (y-b)**2 = r**2 is defn of circle

    sq_dist_to_perimeter = (xp-a)**2 + (yp-b)**2 - r**2
    if sq_dist_to_perimeter < 0:
        # print("sq_dist_to_perimeter < 0 ==> agent center is already within coll zone??")
        return None, None

    sqrt_term = np.sqrt((xp-a)**2 + (yp-b)**2 - r**2)
    xnum1 = r**2 * (xp-a)
    xnum2 = r*(yp-b)*sqrt_term

    ynum1 = r**2 * (yp-b)
    ynum2 = r*(xp-a)*sqrt_term

    den = (xp-a)**2 + (yp-b)**2

    # pt1, pt2 are the tangent pts on the circle perimeter
    pt1 = np.array([(xnum1 + xnum2)/den + a, (ynum1 - ynum2)/den + b])
    pt2 = np.array([(xnum1 - xnum2)/den + a, (ynum1 + ynum2)/den + b])

    # vec1, vec2 are the vecs from (xp,yp) to the tangent pts on the circle perimeter
    vec1 = pt1 - np.array([xp, yp])
    vec2 = pt2 - np.array([xp, yp])

    return vec1, vec2