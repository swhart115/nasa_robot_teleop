
import numpy
from math import pi, sqrt, cos, sin, atan2, acos, asin, fsum
import tf

import rospy

from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion

from PyKDL import *


def normalize_vector(v) :
  m = sqrt(fsum([x*x for x in v]))
  return [x/m for x in v]

def fromTf(tf):
    position, quaternion = tf
    x, y, z = position
    Qx, Qy, Qz, Qw = quaternion
    return Frame(Rotation.Quaternion(Qx, Qy, Qz, Qw),
                 Vector(x, y, z))

def toTf(f):
    return ((f.p[0], f.p[1], f.p[2]), f.M.GetQuaternion())

def fromMsg(p):
    return Frame(Rotation.Quaternion(p.orientation.x,
                                     p.orientation.y,
                                     p.orientation.z,
                                     p.orientation.w),
                 Vector(p.position.x, p.position.y, p.position.z))

def toMsg(f):
    p = Pose()
    p.position.x = f.p[0]
    p.position.y = f.p[1]
    p.position.z = f.p[2]
    q = normalize_vector(f.M.GetQuaternion())
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

def fromMatrix(m):
    return Frame(Rotation(m[0,0], m[0,1], m[0,2],
                          m[1,0], m[1,1], m[1,2],
                          m[2,0], m[2,1], m[2,2]),
                 Vector(m[0,3], m[1, 3], m[2, 3]))

def toMatrix(f):
    return numpy.array([[f.M[0,0], f.M[0,1], f.M[0,2], f.p[0]],
                        [f.M[1,0], f.M[1,1], f.M[1,2], f.p[1]],
                        [f.M[2,0], f.M[2,1], f.M[2,2], f.p[2]],
                        [0,0,0,1]])


def toPose(x, q) :
    p = Pose()
    p.position.x = x[0]
    p.position.y = x[1]
    p.position.z = x[2]
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

def fromCameraParams(cv, rvec, tvec):
    m = numpy.array([ [ 0, 0, 0, tvec[0,0] ],
                      [ 0, 0, 0, tvec[1,0] ],
                      [ 0, 0, 0, tvec[2,0] ],
                      [ 0, 0, 0, 1.0       ] ], dtype = numpy.float32)
    cv.Rodrigues2(rvec, m[:3,:3])
    return fromMatrix(m)

def mag(v):
    mag2 = sum(n * n for n in v)
    return sqrt(mag2)

def sign(val):
    if val > 0:
        return 1
    elif val < 0:
        return -1
    else:
        return 0

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        if mag == 0.: return v
        v = tuple(n / mag for n in v)
    return v

def axis_to_q(v, roll_offset=0):
    x, y, z = v
    roll=pi + roll_offset
    rad = sqrt(x*x+y*y+z*z)
    pitch = atan2(-z,sqrt(x*x+y*y)) #acos(sqrt(x*x+y*y)/rad)
    yaw = atan2(y,x)
    return rpy_to_q((roll,pitch,yaw))

def rpy_to_q(v):
    r, p, y = [x/2 for x in v]
    w = cos(r)*cos(p)*cos(y)+sin(r)*sin(p)*sin(y)
    x = sin(r)*cos(p)*cos(y)-cos(r)*sin(p)*sin(y)
    y = cos(r)*sin(p)*cos(y)+sin(r)*cos(p)*sin(y)
    z = cos(r)*cos(p)*sin(y)-sin(r)*sin(p)*cos(y)
    return x, y, z, w

def q_to_axisangle(q):
    w, v = q[0], q[1:]
    theta = acos(w) * 2.0
    return normalize(v), theta

def q_to_rpy(q):
    x,y,z,w = q
    wx,x = normalize((w,x))
    wy,y = normalize((w,y))
    wz,z = normalize((w,z))
    return sign(x)*2*acos(wx), sign(y)*2*acos(wy), sign(z)*2*acos(wz)


# class PoseMath(object):

#     def __init__(self, msg):
#         self.msg = msg

#     @staticmethod
#     def fromMatrix(m):
#         (x, y, z) = (m[0, 3], m[1, 3], m[2, 3])
#         q = transformations.quaternion_from_matrix(m)
#         return PoseMath(Pose(Point(x, y, z), Quaternion(*q)))

#     @staticmethod
#     def fromTf(tf):
#         position, quaternion = tf
#         return PoseMath(Pose(Point(*position), Quaternion(*quaternion)))

#     @staticmethod
#     def fromEuler(x, y, z, Rx, Ry, Rz):
#         q = transformations.quaternion_from_euler(Rx, Ry, Rz)
#         return PoseMath(Pose(Point(x, y, z), Quaternion(*q)))

#     @staticmethod
#     def fromCameraParams(self, cv, rvec, tvec):
#         m = numpy.array([ [ 0, 0, 0, tvec[0,0] ],
#                           [ 0, 0, 0, tvec[1,0] ],
#                           [ 0, 0, 0, tvec[2,0] ],
#                           [ 0, 0, 0, 1.0       ] ], dtype = numpy.float32)
#         cv.Rodrigues2(rvec, m[:3,:3])
#         return self.fromMatrix(m)

#     # Operators

#     def __mul__(self, other):
#         m = numpy.dot(self.asMatrix(), other.asMatrix())
#         return PoseMath.fromMatrix(m)

#     def __invert__(self):
#         inv = numpy.linalg.inv(self.asMatrix())
#         return PoseMath.fromMatrix(inv)

#     # Representations

#     def __repr__(self):
#         return repr(self.msg)

#     def asMessage(self):
#         """ Return the pose as a ROS ``Pose`` message """
#         return self.msg

#     def asMatrix(self):
#         """ Return a numpy 4x4 array for the pose. """
#         translation = (self.msg.position.x, self.msg.position.y, self.msg.position.z)
#         rotation = (self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w)
#         return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

#     def asEuler(self):
#         """ Return a tuple (x, y, z, Rx, Ry, Rz) for the pose. """
#         tmp = transformations.euler_from_quaternion((self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w))
#         return (self.msg.position.x, self.msg.position.y, self.msg.position.z, tmp[0], tmp[1], tmp[2])

#     def asTf(self):
#         """ Return a tuple (position, quaternion) for the pose. """
#         p = self.msg.position
#         q = self.msg.orientation
#         return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

