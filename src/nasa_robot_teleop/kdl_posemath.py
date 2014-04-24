from geometry_msgs.msg import Pose, Point, Quaternion
from tf import transformations
import tf
import rospy
import numpy

from PyKDL import *

def fromTf(tf):
    """
    :param tf: :class:`tf.Transformer` transform
    :type tf: tuple (translation, quaternion)
    :return: New :class:`PyKDL.Frame` object

    Convert a pose returned by :meth:`tf.Transformer.lookupTransform` to a :class:`PyKDL.Frame`.

    .. doctest::

        >>> import rospy
        >>> import tf
        >>> import geometry_msgs.msg
        >>> t = tf.Transformer(True, rospy.Duration(10.0))
        >>> m = geometry_msgs.msg.TransformStamped()
        >>> m.header.frame_id = 'THISFRAME'
        >>> m.child_frame_id = 'CHILD'
        >>> m.transform.translation.x = 668.5
        >>> m.transform.rotation.w = 1.0
        >>> t.setTransform(m)
        >>> t.lookupTransform('THISFRAME', 'CHILD', rospy.Time(0))
        ((668.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        >>> import tf_conversions.posemath as pm
        >>> p = pm.fromTf(t.lookupTransform('THISFRAME', 'CHILD', rospy.Time(0)))
        >>> print pm.toMsg(p * p)
        position: 
          x: 1337.0
          y: 0.0
          z: 0.0
        orientation: 
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0

    """

    position, quaternion = tf
    x, y, z = position
    Qx, Qy, Qz, Qw = quaternion
    return Frame(Rotation.Quaternion(Qx, Qy, Qz, Qw), 
                 Vector(x, y, z))

def toTf(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a tuple (position, quaternion) for the pose.
    """

    return ((f.p[0], f.p[1], f.p[2]), f.M.GetQuaternion())

# to and from pose message
def fromMsg(p):
    """
    :param p: input pose
    :type p: :class:`geometry_msgs.msg.Pose`
    :return: New :class:`PyKDL.Frame` object

    Convert a pose represented as a ROS Pose message to a :class:`PyKDL.Frame`.
    """
    return Frame(Rotation.Quaternion(p.orientation.x,
                                     p.orientation.y,
                                     p.orientation.z,
                                     p.orientation.w),  
                 Vector(p.position.x, p.position.y, p.position.z))

def toMsg(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a ROS Pose message for the Frame f.

    """
    p = Pose()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = f.M.GetQuaternion()
    p.position.x = f.p[0]
    p.position.y = f.p[1]
    p.position.z = f.p[2]
    return p


# to and from matrix
def fromMatrix(m):
    """
    :param m: input 4x4 matrix
    :type m: :func:`numpy.array`
    :return: New :class:`PyKDL.Frame` object

    Convert a pose represented as a 4x4 numpy array to a :class:`PyKDL.Frame`.

    """
    return Frame(Rotation(m[0,0], m[0,1], m[0,2],
                          m[1,0], m[1,1], m[1,2],
                          m[2,0], m[2,1], m[2,2]),  
                 Vector(m[0,3], m[1, 3], m[2, 3]))
    
def toMatrix(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a numpy 4x4 array for the Frame F.
    """
    return numpy.array([[f.M[0,0], f.M[0,1], f.M[0,2], f.p[0]],
                        [f.M[1,0], f.M[1,1], f.M[1,2], f.p[1]],
                        [f.M[2,0], f.M[2,1], f.M[2,2], f.p[2]],
                        [0,0,0,1]])


def toPose(x, q) :
    """
    :param x: input position
    :param 1: input quaternion
    :type p: output pose

    Return a ROS pose from python lists.
    """
    p = Pose()
    p.position.x = x[0]
    p.position.y = x[1]
    p.position.z = x[2]
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

# from camera parameters
def fromCameraParams(cv, rvec, tvec):
    """
    :param cv: OpenCV module
    :param rvec: A Rodrigues rotation vector - see :func:`Rodrigues2`
    :type rvec: 3x1 :class:`CvMat`
    :param tvec: A translation vector 
    :type tvec: 3x1 :class:`CvMat`
    :return: New :class:`PyKDL.Frame` object
    
    For use with :func:`FindExtrinsicCameraParams2`::

        import cv
        import tf_conversions.posemath as pm
        ...
        rvec = cv.CreateMat(3, 1, cv.CV_32FC1)
        tvec = cv.CreateMat(3, 1, cv.CV_32FC1)
        cv.FindExtrinsicCameraParams2(model, corners, intrinsic_matrix, kc, rvec, tvec)
        pose = pm.fromCameraParams(cv, rvec, tvec)

    """
    m = numpy.array([ [ 0, 0, 0, tvec[0,0] ],
                      [ 0, 0, 0, tvec[1,0] ], 
                      [ 0, 0, 0, tvec[2,0] ], 
                      [ 0, 0, 0, 1.0       ] ], dtype = numpy.float32)
    cv.Rodrigues2(rvec, m[:3,:3])
    return fromMatrix(m)


class PoseMath(object):
    """
    :param msg: ROS Pose message


    Poses can be concatenated using ``*``:

    .. doctest::

        >>> import tf_conversions.posemath as pm
        >>> from math import pi
        >>> trans = PoseMath.fromEuler(1, 2, 3, 0, 0, 0)
        >>> rotate = PoseMath.fromEuler(0, 0, 0, pi / 2, 0, 0)
        >>> print trans * rotate
        position: 
          x: 1.0
          y: 2.0
          z: 3.0
        orientation: 
          x: 0.707106781187
          y: 0.0
          z: 0.0
          w: 0.707106781187
        >>> print rotate * trans
        position: 
          x: 1.0
          y: -3.0
          z: 2.0
        orientation: 
          x: 0.707106781187
          y: 0.0
          z: 0.0
          w: 0.707106781187

    and inverted using ``~``:

    .. doctest::

        >>> import tf_conversions.posemath as pm
        >>> trans = PoseMath.fromEuler(1, 2, 3, 0, 0, 0)
        >>> print ~trans
        position: 
          x: -1.0
          y: -2.0
          z: -3.0
        orientation: 
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0

    """

    # Initializers

    def __init__(self, msg):
        self.msg = msg

    @staticmethod
    def fromMatrix(m):
        """
        :param m: 4x4 array
        :type m: :func:`numpy.array`
        :return: New PoseMath object

        Return a PoseMath object initialized from 4x4 matrix m

        .. doctest::

            >>> import numpy
            >>> import tf_conversions.posemath as pm
            >>> print PoseMath.fromMatrix(numpy.identity(4))
            position: 
              x: 0.0
              y: 0.0
              z: 0.0
            orientation: 
              x: 0.0
              y: 0.0
              z: 0.0
              w: 1.0

        """
        (x, y, z) = (m[0, 3], m[1, 3], m[2, 3])
        q = transformations.quaternion_from_matrix(m)
        return PoseMath(Pose(Point(x, y, z), Quaternion(*q)))


    @staticmethod
    def fromTf(tf):
        """
        :param tf: tf transform
        :type tf: tuple (translation, quaternion)
        :return: New PoseMath object

        Return a PoseMath object initialized from a :mod:`tf` transform, as returned by :meth:`tf.Transformer.lookupTransform`.

        .. doctest::

            >>> import rospy
            >>> import tf
            >>> import geometry_msgs.msg
            >>> t = tf.Transformer(True, rospy.Duration(10.0))
            >>> m = geometry_msgs.msg.TransformStamped()
            >>> m.header.frame_id = 'THISFRAME'
            >>> m.child_frame_id = 'CHILD'
            >>> m.transform.translation.x = 668.5
            >>> m.transform.rotation.w = 1.0
            >>> t.setTransform(m)
            >>> t.lookupTransform('THISFRAME', 'CHILD', rospy.Time(0))
            ((668.5, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            >>> import tf_conversions.posemath as pm
            >>> p = PoseMath.fromTf(t.lookupTransform('THISFRAME', 'CHILD', rospy.Time(0)))
            >>> print p * p
            position: 
              x: 1337.0
              y: 0.0
              z: 0.0
            orientation: 
              x: 0.0
              y: 0.0
              z: 0.0
              w: 1.0

        """
        position, quaternion = tf
        return PoseMath(Pose(Point(*position), Quaternion(*quaternion)))

    @staticmethod
    def fromEuler(x, y, z, Rx, Ry, Rz):
        """
        :param x: x translation
        :type x: float 
        :param y: y translation
        :type y: float 
        :param z: z translation
        :type z: float 
        :param Rx: rotation around x-axis in radians
        :type Rx: float 
        :param Ry: rotation around y-axis in radians
        :type Ry: float 
        :param Rz: rotation around z-axis in radians
        :type Rz: float 
        :return: New PoseMath object

        Return a PoseMath object initialized from translation (x, y, z) and Euler rotation (Rx, Ry, Rz).

        .. doctest::

            >>> import tf_conversions.posemath as pm
            >>> from math import pi
            >>> print PoseMath.fromEuler(1, 2, 3, 0, pi / 2, 0)
            position: 
              x: 1
              y: 2
              z: 3
            orientation: 
              x: 0.0
              y: 0.707106781187
              z: 0.0
              w: 0.707106781187


        """
        q = transformations.quaternion_from_euler(Rx, Ry, Rz)
        return PoseMath(Pose(Point(x, y, z), Quaternion(*q)))

    @staticmethod
    def fromCameraParams(self, cv, rvec, tvec):
        """
        :param cv: OpenCV module
        :param rvec: A Rodrigues rotation vector - see :func:`Rodrigues2`
        :type rvec: 3x1 :class:`CvMat`
        :param tvec: A translation vector 
        :type tvec: 3x1 :class:`CvMat`
        :return: New PoseMath object
        
        For use with :func:`FindExtrinsicCameraParams2`::

            import cv
            import tf_conversions.posemath as pm
            ...
            rvec = cv.CreateMat(3, 1, cv.CV_32FC1)
            tvec = cv.CreateMat(3, 1, cv.CV_32FC1)
            cv.FindExtrinsicCameraParams2(model, corners, intrinsic_matrix, kc, rvec, tvec)
            pose = PoseMath.fromCameraParams(cv, rvec, tvec)

        """
        m = numpy.array([ [ 0, 0, 0, tvec[0,0] ],
                          [ 0, 0, 0, tvec[1,0] ], 
                          [ 0, 0, 0, tvec[2,0] ], 
                          [ 0, 0, 0, 1.0       ] ], dtype = numpy.float32)
        cv.Rodrigues2(rvec, m[:3,:3])
        return self.fromMatrix(m)

    # Operators

    def __mul__(self, other):
        m = numpy.dot(self.asMatrix(), other.asMatrix())
        return PoseMath.fromMatrix(m)

    def __invert__(self):
        inv = numpy.linalg.inv(self.asMatrix())
        return PoseMath.fromMatrix(inv)

    # Representations

    def __repr__(self):
        return repr(self.msg)

    def asMessage(self):
        """ Return the pose as a ROS ``Pose`` message """
        return self.msg

    def asMatrix(self):
        """ Return a numpy 4x4 array for the pose. """
        translation = (self.msg.position.x, self.msg.position.y, self.msg.position.z)
        rotation = (self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w)
        return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))        

    def asEuler(self):
        """ Return a tuple (x, y, z, Rx, Ry, Rz) for the pose. """
        tmp = transformations.euler_from_quaternion((self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w))
        return (self.msg.position.x, self.msg.position.y, self.msg.position.z, tmp[0], tmp[1], tmp[2])

    def asTf(self):
        """ Return a tuple (position, quaternion) for the pose. """
        p = self.msg.position
        q = self.msg.orientation
        return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
