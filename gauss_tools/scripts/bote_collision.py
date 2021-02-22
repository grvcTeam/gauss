#!/usr/bin/env python
import math

def calculate_intersection(p1, p2, q1, q2):
    """Return the 2D geometrical intersection of segments [p1, p2] and [q1, q2]
    
    inputs:
    - p1, p2: 2D points that define the first segment
    - q1, q2: 2D points that define the second segment
    """
    num_s = float((p1[0] - q1[0]) * (q1[1] - q2[1]) - (p1[1] - q1[1]) * (q1[0] - q2[0]))
    den_s = float((p1[0] - p2[0]) * (q1[1] - q2[1]) - (p1[1] - p2[1]) * (q1[0] - q2[0]))
    if (den_s == 0):
        # Intersection undefined
        return (float("nan"), float("nan"))

    s = num_s / den_s
    if (s < 0) or (s > 1.0):
        # No intersection
        return (float("nan"), float("nan"))

    # Point of intersection
    x = p1[0] + s * (p2[0] - p1[0])
    y = p1[1] + s * (p2[1] - p1[1])
    return (x,y)

def calculate_delta_t(initial, final, speed):
    """ Return estimated time from initial to final, given a speed

    inputs:
    - initial: 2D point defining the initial position
    - final:   2D point defining the final position
    - speed:   speed used to calculate delta_t
    """
    delta_x = final[0] - initial[0]
    delta_y = final[1] - initial[1]
    distance = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    return distance / speed

def main():
    # TODO: input data from file?
    A1 = ( 0,  0)  # [m]
    A2 = (+4, +3)  # [m]
    B1 = ( 0, +3)  # [m]
    B2 = (+4,  0)  # [m]
    vA = 5  # [m/s]
    vB = 1  # [m/s]

    collision = calculate_intersection(A1, A2, B1, B2)
    print('Possible collision in coordinates {}m'.format(collision))

    dt_A = calculate_delta_t(A1, collision, vA)
    dt_B = calculate_delta_t(B1, collision, vB)
    print('dt_A = {}s, dt_B = {}s'.format(dt_A, dt_B))

    if (dt_A > dt_B):
        dt = dt_A - dt_B
        print('B should start {}s after A'.format(dt))
    else:
        dt = dt_B - dt_A
        print('A should start {}s after B'.format(dt))

if __name__ == '__main__':
    main()
