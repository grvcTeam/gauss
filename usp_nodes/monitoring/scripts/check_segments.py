#!/usr/bin/python
import math
import copy
import rospy
from geometry_msgs.msg import Vector3
from gauss_msgs.msg import Waypoint

def clamp(x, min_x, max_x):
  if min_x > max_x:
    raise ValueError('min_x[{}] > max_x[{}]'.format(min_x, max_x))
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

# Signed Distance Fucntion from P to:
# a sphere centered in C with radius R
def sdSphere(p, c, r):
  cp = vector_from_point_to_point(c, p)
  return length(cp) - r

# Signed Distance Fucntion from P to:
# a segment (A,B) with some radius R
def sdSegment(p, a, b, r = 0):
  ap = vector_from_point_to_point(a, p)
  ab = vector_from_point_to_point(a, b)
  h = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0)
  return length(ap - ab*h) - r

class Segment(object):
  def __init__(self, a, b):
    self.point_a = a
    self.point_b = b
    if self.point_a == self.point_b:
      print('a == b == {}'.format(self.point_a))
    self.t_a = a.stamp.to_sec()
    self.t_b = b.stamp.to_sec()
    if self.t_a >= self.t_b:
      print('t_a[{}] >= t_b[{}]'.format(self.t_a, self.t_b))

  def __str__(self):
    result = ''
    result += '[({}); ({})]'.format(self.point_a, self.point_b)
    return result

  def point_at_time(self, t):
    if t < self.t_a:
      print('t[{}] < t_a[{}]'.format(t, self.t_a))
      return self.point_a
    if t > self.t_b:
      print('t[{}] > t_b[{}]'.format(t, self.t_b))
      return self.point_b
    if self.t_a == self.t_b:
      print('t_a == t_b == {}'.format(self.t_a))
      return self.point_a

    u = (t - self.t_a) / (self.t_b - self.t_a)
    point = Waypoint()
    point.x = self.point_a.x * (1.0 - u) + self.point_b.x * u
    point.y = self.point_a.y * (1.0 - u) + self.point_b.y * u
    point.z = self.point_a.z * (1.0 - u) + self.point_b.z * u
    point.stamp = rospy.Time.from_sec(t)
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
      print('u[{}] < 0, clamping!'.format(u))
      u = 0

    if u > 1:
      print('u[{}] > 1, clamping!'.format(u))
      u = 1

    d = delta(first, second)
    delta_x = d[0].x * (1 - u) + d[1].x * u
    delta_y = d[0].y * (1 - u) + d[1].y * u
    delta_z = d[0].z * (1 - u) + d[1].z * u
    return delta_x**2 + delta_y**2 + delta_z**2

def quadratic_roots(a, b, c):  
  if (a == 0) and (b == 0) and (c == 0):
    print('a = b = c = 0, any number is a solution!')
    return (float('nan'), float('nan'))
  if (a == 0) and (b == 0) and (c != 0):
    print('a = b = 0, there is no solution!')
    return (float('nan'), float('nan')) 
  if (a == 0) and (b != 0):
    print('a = 0, non quadratic!')
    return (-c/b, -c/b)

  d = b*b - 4*a*c
  if d < 0:
    print('d = [{}], complex solutions!'.format(d))
    return (float('nan'), float('nan'))

  e = math.sqrt(d)
  return ((-b - e)/(2*a), (-b + e)/(2*a))

class CheckSegmentsLossResults(object):
  def __init__(self, first, second):
    self.first = copy.deepcopy(first)
    self.second = copy.deepcopy(second)
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

def checkUnifiedSegmentsLoss(first, second, s_threshold):
  # print('checkUnifiedSegmentsLoss:')
  # print(first.point_a)
  # print(first.point_b)
  # print('___________')
  # print(second.point_a)
  # print(second.point_b)
  d = delta(first, second)
  # print(d)
  C_x = d[0].x ** 2
  C_y = d[0].y ** 2
  C_z = d[0].z ** 2
  B_x = 2 * (d[0].x * d[1].x - C_x)
  B_y = 2 * (d[0].y * d[1].y - C_y)
  B_z = 2 * (d[0].z * d[1].z - C_z)
  A_x = (d[1].x - d[0].x) ** 2
  A_y = (d[1].y - d[0].y) ** 2
  A_z = (d[1].z - d[0].z) ** 2
  A = A_x + A_y + A_z
  B = B_x + B_y + B_z
  C = C_x + C_y + C_z

  if A == 0:
    print('A = 0')
    if B >= 0:
      u_min = 0
      t_min = first.t_a
      s_min = C
    else:
      u_min = 1
      t_min = first.t_b
      s_min = B+C
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

  result = CheckSegmentsLossResults(first, second)
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

def checkSegmentsLoss(first, second, s_threshold):
  # print('checkSegmentsLoss:')
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
    return CheckSegmentsLossResults(first, second)

  p_alpha1 = first.point_at_time(t_alpha)
  p_beta1 = first.point_at_time(t_beta)
  p_alpha2 = second.point_at_time(t_alpha)
  p_beta2 = second.point_at_time(t_beta)
  return checkUnifiedSegmentsLoss(Segment(a=p_alpha1, b=p_beta1), Segment(a=p_alpha2, b=p_beta2), s_threshold)

def main():
  first  = Segment(Waypoint(x = 0, y = 0, z = 0, stamp = rospy.Time(0)), Waypoint(x = 8, y = 6, z = 0, stamp = rospy.Time(10)))
  second = Segment(Waypoint(x = 0, y = 6, z = 0, stamp = rospy.Time(0)), Waypoint(x = 8, y = 0, z = 0, stamp = rospy.Time(10)))
  # first  = Segment(Waypoint(x = 0, y = 5, z = 0, stamp = rospy.Time(0)), Waypoint(x = 8, y = 6, z = 0, stamp = rospy.Time(10)))
  # second = Segment(Waypoint(x = 0, y = 0, z = 0, stamp = rospy.Time(10)), Waypoint(x = 8, y = 0, z = 0, stamp = rospy.Time(12)))

  s_threshold = 1.0  # TODO: param
  print(checkSegmentsLoss(first, second, s_threshold))

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
