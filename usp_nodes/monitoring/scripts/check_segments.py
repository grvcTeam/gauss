#!/usr/bin/python
import math
import rospy
from geometry_msgs.msg import Vector3
from gauss_msgs.msg import Waypoint

def clamp(x, min_x, max_x):
  if min_x > max_x:
    print('min_x[{}] > max_x[{}]'.format(min_x, max_x))
    # TODO

  return max(min(x, max_x), min_x)

class Segment(object):
  def __init__(self, a, b):
    self.point_a = a
    self.point_b = b
    self.t_a = a.stamp.to_sec()
    self.t_b = b.stamp.to_sec()
    if self.t_a >= self.t_b:
      print('t_a[{}] >= t_b[{}]'.format(self.t_a, self.t_b))
      # TODO

  def point_at_time(self, t):
    if t < self.t_a:
      print('t[{}] < t_a[{}]'.format(t, self.t_a))
      # TODO
    if t > self.t_b:
      print('t[{}] > t_b[{}]'.format(t, self.t_b))
      # TODO

    # TODO: division by zero: is_valid flag?
    u = (t - self.t_a) / (self.t_b - self.t_a)
    point = Waypoint()
    point.x = self.point_a.x * (1.0 - u) + self.point_b.x * u
    point.y = self.point_a.y * (1.0 - u) + self.point_b.y * u
    point.z = self.point_a.z * (1.0 - u) + self.point_b.z * u
    point.stamp = rospy.Time.from_sec(t)
    # TODO: use mandatory as is_valid?
    return point

def delta(first, second):
  delta_a = Vector3()
  delta_a.x = second.point_a.x - first.point_a.x
  delta_a.y = second.point_a.y - first.point_a.y
  delta_a.z = second.point_a.z - first.point_a.z
  delta_b = Vector3()
  delta_b.x = second.point_b.x - first.point_b.x
  delta_b.y = second.point_b.y - first.point_b.y
  delta_b.z = second.point_b.z - first.point_b.z
  return (delta_a, delta_b)

def sq_distance(first, second, u):
    if u < 0:
      print('u[{}] < 0'.format(u))
      # TODO

    if u > 1:
      print('u[{}] > 1'.format(u))
      # TODO

    d = delta(first, second)
    delta_x = d[0].x * (1 - u) + d[1].x * u
    delta_y = d[0].y * (1 - u) + d[1].y * u
    delta_z = d[0].z * (1 - u) + d[1].z * u
    return delta_x**2 + delta_y**2 + delta_z**2

def quadratic_roots(a, b, c):  
  if (a == 0):
    print('a = 0, non quadratic!')
    return (float('nan'), float('nan'))

  d = b*b - 4*a*c
  if d < 0:
    print('d = [{}], complex solutions!'.format(d))
    return (float('nan'), float('nan'))

  e = math.sqrt(d)
  return ((-b - e)/(2*a), (-b + e)/(2*a))

def checkUnifiedSegments(first, second, s_threshold = 0):
  print('checkUnifiedSegments:')
  # print(first.point_a)
  # print(first.point_b)
  # print('___________')
  # print(second.point_a)
  # print(second.point_b)
  d = delta(first, second)
  # print(d)
  A_x = (d[1].x - d[0].x) ** 2
  A_y = (d[1].y - d[0].y) ** 2
  A_z = (d[1].z - d[0].z) ** 2
  B_x = 2 * (d[0].x * d[1].x - d[0].x ** 2)
  B_y = 2 * (d[0].y * d[1].y - d[0].y ** 2)
  B_z = 2 * (d[0].z * d[1].z - d[0].z ** 2)
  # TODO: reuse C_* in B_* and A_*?
  C_x = d[0].x ** 2
  C_y = d[0].y ** 2
  C_z = d[0].z ** 2
  A = A_x + A_y + A_z
  B = B_x + B_y + B_z
  C = C_x + C_y + C_z

  if A == 0:
    print('A = 0')
    s_min = min(C, B+C)
    print(s_min)
  else:
    u_star = -0.5 * B / A
    t_star = first.t_a * (1-u_star) + first.t_b * u_star
    print(u_star)
    print(t_star)
    print(sq_distance(first, second, u_star))
    u_prime = clamp(u_star, 0, 1)
    t_prime = first.t_a * (1-u_prime) + first.t_b * u_prime
    s_min = sq_distance(first, second, u_prime)
    print(u_prime)
    print(t_prime)
    print(s_min)

  if s_min > s_threshold:
    print('s_min[{}] > s_threshold[{}]'.format(s_min, s_threshold))
    # TODO
    return False

  u_crossing = quadratic_roots(A, B, C - s_threshold)
  t_crossing_a = first.t_a * (1-u_crossing[0]) + first.t_b * u_crossing[0]
  t_crossing_b = first.t_a * (1-u_crossing[1]) + first.t_b * u_crossing[1]
  print(u_crossing)
  print(t_crossing_a, t_crossing_b)
  # TODO: clamp u_crossing
  # TODO: Calculate t_crossing and new segments to return
  return True

def checkSegments(first, second):
  # print('checkSegments:')
  # print(first.point_a)
  # print(first.point_b)
  # print('___________')
  # print(second.point_a)
  # print(second.point_b)
  t_a1 = first.t_a
  t_b1 = first.t_b
  t_a2 = second.t_a
  t_b2 = second.t_b
  t_alpha = max(t_a1, t_a2)
  t_beta  = min(t_b1, t_b2)
  if t_alpha > t_beta:
    print('t_alpha[{}] > t_beta[{}]'.format(t_alpha, t_beta))
    # TODO
    return False

  p_alpha1 = first.point_at_time(t_alpha)
  p_beta1 = first.point_at_time(t_beta)
  p_alpha2 = second.point_at_time(t_alpha)
  p_beta2 = second.point_at_time(t_beta)
  return checkUnifiedSegments(Segment(a=p_alpha1, b=p_beta1), Segment(a=p_alpha2, b=p_beta2), 1.0)

def main():
  first  = Segment(Waypoint(x = 0, y = 0, z = 0, stamp = rospy.Time(0)), Waypoint(x = 8, y = 6, z = 0, stamp = rospy.Time(10)))
  second = Segment(Waypoint(x = 0, y = 6, z = 0, stamp = rospy.Time(0)), Waypoint(x = 8, y = 0, z = 0, stamp = rospy.Time(10)))
  # first  = Segment(Waypoint(x = 0, y = 5, z = 0, stamp = rospy.Time(0)), Waypoint(x = 8, y = 6, z = 0, stamp = rospy.Time(10)))
  # second = Segment(Waypoint(x = 0, y = 0, z = 0, stamp = rospy.Time(10)), Waypoint(x = 8, y = 0, z = 0, stamp = rospy.Time(12)))

  # print(first.point_at_time(0.0))
  print(checkSegments(first, second))

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
