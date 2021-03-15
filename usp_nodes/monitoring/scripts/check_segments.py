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

def dot(u, v):
  return u.x*v.x + u.y*v.y + u.z*v.z

def length(u):
  return math.sqrt(u.x*u.x + u.y*u.y + u.z*u.z)

def vector_from_point_to_point(a, b):
  ab = Vector3()
  ab.x = b.x - a.x
  ab.y = b.y - a.y
  ab.z = b.z - a.z
  return ab

# float sdSphere( vec3 p, float s )
# {
#   return length(p)-s;
# }
def sdSphere(p, c, r):
  cp = vector_from_point_to_point(c, p)
  return length(cp) - r

# float sdCapsule( vec3 p, vec3 a, vec3 b, float r )
# {
#   vec3 pa = p - a, ba = b - a;
#   float h = clamp( dot(pa,ba)/dot(ba,ba), 0.0, 1.0 );
#   return length( pa - ba*h ) - r;
# }
# Signed Distance Fucntion from P to:
# a segment (A,B) with some radius R
def sdCapsule(p, a, b, r):
  ap = vector_from_point_to_point(a, p)
  ab = vector_from_point_to_point(a, b)
  h = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0 )
  return length(ap - ab*h) - r

class Segment(object):
  def __init__(self, a, b):
    self.point_a = a
    self.point_b = b
    # TODO: What if a == b?
    self.t_a = a.stamp.to_sec()
    self.t_b = b.stamp.to_sec()
    if self.t_a >= self.t_b:
      print('t_a[{}] >= t_b[{}]'.format(self.t_a, self.t_b))
      # TODO

  def __str__(self):
    result = ''
    result += '[({}); ({})]'.format(self.point_a, self.point_b)
    return result

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

class CheckSegmentsResults(object):
  def __init__(self, first, second):
    self.first = first
    self.second = second
    self.t_min = float('nan')
    self.s_min = float('nan')
    self.t_crossing_0 = float('nan')
    self.t_crossing_1 = float('nan')
    self.threshold_is_violated = False

  def __str__(self):
    result = ''
    result += 'first = {}\n'.format(self.first)
    result += 'second = {}\n'.format(self.second)
    result += 't_min = {}\n'.format(self.t_min)
    result += 's_min = {}\n'.format(self.s_min)
    result += 't_crossing_0 = {}\n'.format(self.t_crossing_0)
    result += 't_crossing_1 = {}\n'.format(self.t_crossing_1)
    result += 'threshold_is_violated = {}\n'.format(self.threshold_is_violated)
    return result

def checkUnifiedSegments(first, second, s_threshold = 0):
  # print('checkUnifiedSegments:')
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
    # TODO: t_min?
    s_min = min(C, B+C)
    # print(s_min)
  else:
    u_star = -0.5 * B / A
    t_star = first.t_a * (1-u_star) + first.t_b * u_star
    # print(u_star)
    # print(t_star)
    # print(sq_distance(first, second, u_star))
    u_min = clamp(u_star, 0, 1)
    t_min = first.t_a * (1-u_min) + first.t_b * u_min
    s_min = sq_distance(first, second, u_min)
    # print(u_min)
    # print(t_min)
    # print(s_min)

  result = CheckSegmentsResults(first, second)
  result.t_min = t_min
  result.s_min = s_min

  if s_min > s_threshold:
    print('s_min[{}] > s_threshold[{}]'.format(s_min, s_threshold))
    # TODO
    result.threshold_is_violated = False
    return result

  u_bar = quadratic_roots(A, B, C - s_threshold)
  t_bar_0 = first.t_a * (1-u_bar[0]) + first.t_b * u_bar[0]
  t_bar_1 = first.t_a * (1-u_bar[1]) + first.t_b * u_bar[1]
  # print(u_bar)
  # print(t_bar_0, t_bar_1)
  # TODO: Calculate t_crossing and new segments to return
  u_crossing_0 = clamp(u_bar[0], 0, 1)
  u_crossing_1 = clamp(u_bar[1], 0, 1)
  t_crossing_0 = first.t_a * (1-u_crossing_0) + first.t_b * u_crossing_0
  t_crossing_1 = first.t_a * (1-u_crossing_1) + first.t_b * u_crossing_1
  # print(u_crossing_0, u_crossing_1)
  # print(t_crossing_0, t_crossing_1)
  first_in_conflict = Segment(first.point_at_time(t_crossing_0), first.point_at_time(t_crossing_1))
  second_in_conflict = Segment(second.point_at_time(t_crossing_0), second.point_at_time(t_crossing_1))

  result.first = first_in_conflict
  result.second = second_in_conflict
  result.t_crossing_0 = t_crossing_0
  result.t_crossing_1 = t_crossing_1
  result.threshold_is_violated = True
  return result

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
    return CheckSegmentsResults(first, second)

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
