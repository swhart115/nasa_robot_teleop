#!/usr/bin/env python
import sys
from random import random

import rospy
import roslib; roslib.load_manifest('marker_templates')
import tf
from dynamic_reconfigure.server import Server
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import PyKDL as kdl

from MarkerTemplate import MarkerTemplate
from marker_templates.cfg import WaypointConfig
from util import *

from copy import deepcopy

# groovy changed how scaling affects interactive markers
import rosparam
groovy = '1.9.41'
if rosparam.get_param('/rosversion') >= groovy:
    scaleFactorX = scaleFactorY = scaleFactorZ = 1
else:
    scaleFactorX = 2
    scaleFactorY = 10
    scaleFactorZ = 5

# TODO straight port from C++, python voodoo can simplify this greatly
class Waypoint(MarkerTemplate):
    def __init__(self, server, key, id_):
        MarkerTemplate.__init__(self, server, key, id_)
        self.setup()

    def setup(self):
        self._color = ColorRGBA(random(), random(), random(), 1.0)
        self._tf_listener = tf.TransformListener()
        self._pathHandler = MenuHandler()
        self._waypoints = []
        self._wpStack = []
        self._pathMarkers = []
        self._frame_id = "/world"
        self._waypointPublishTopic = "/datastore/waypoints"
        self._footstepPublishTopic = "/datastore/operator_adjusted/footsteps"
        self._footstepSubscribeTopic = "/operator/footsteps"
        self._height = 0.0
        self._markerData = None
        self._planningFootPath = False
        self._feetAdjustMenuID = 0
        self._feetHeightMenuID = 0
        self._feetPublishMenuID = 0
        self._syncToActualMenuID = 0

        self._rightFootActualPose = PoseStamped()
        self._leftFootActualPose = PoseStamped()

        self._dserver = Server(WaypointConfig, self.dynamicReconfigureCb)
        # attach menu handler callbacks
        self.menu_handler.insert("Add Waypoint", callback=self.addWaypointCb)
        self.menu_handler.setCheckState( self.menu_handler.insert( "Plan Footstep Path", callback=self.planFeetPathCb ), MenuHandler.UNCHECKED )
        self._feetAdjustMenuID = self.menu_handler.insert("Adjust Feet", callback=self.adjustFeet)
        self._syncToActualMenuID = self.menu_handler.insert("Sync to Actual Feet", callback=self.syncActualPoses)
        self.menu_handler.insert("Publish Waypoints", callback=self.publishWaypoints)
        self._feetPublishMenuID = self.menu_handler.insert("Publish Footsteps", callback=self.publishFootseps)
        self.menu_handler.insert("Delete THIS Waypoint", callback=self.deleteWaypointCb)
        self._feetHeightMenuID = self.menu_handler.insert("Toggle Height Control", callback=self.addHeightControl)
        self._pathHandler.insert("Add Waypoint", callback=self.pathCb)

        self.menu_handler.setVisible(self._feetAdjustMenuID, True)
        self.menu_handler.setVisible(self._syncToActualMenuID, False)
        self.menu_handler.setVisible(self._feetPublishMenuID, False)
        self.menu_handler.setVisible(self._feetHeightMenuID, False)

        # TODO: temporarily coding all waypoint sets to the same publisher
        # self._publisher = rospy.Publisher("/datastore/path" + str(self.id), MarkerArray)
        self._waypointPublisher = rospy.Publisher(self._waypointPublishTopic, MarkerArray)
        self._footstepPublisher = rospy.Publisher(self._footstepPublishTopic, MarkerArray)

        rospy.Subscriber(self._footstepSubscribeTopic, MarkerArray, self.footstepInCB)

        # rospy.Subscriber("/v1/RightFoot/pose", Odometry, self.rightFootPoseCallback)
        # rospy.Subscriber("/v1/LeftFoot/pose", Odometry, self.leftFootPoseCallback)

        self._leftFootMesh = "package://marker_templates/resources/models/left_foot.stl"
        self._rightFootMesh = "package://marker_templates/resources/models/right_foot.stl"

        self._footstepInteractiveMarkers = {}
        self._footstepHeightControls = {}

        self._TwoFeetPoses = {}
        self._TwoFeetPoses['left'] = Pose()
        self._TwoFeetPoses['right'] = Pose()

        self._TwoFeetPoses['left'].position.x = 0.25
        self._TwoFeetPoses['left'].position.y = 0.15
        self._TwoFeetPoses['left'].position.z = 0.09

        self._TwoFeetPoses['left'].orientation.x = -0.7067963027413156
        self._TwoFeetPoses['left'].orientation.y = 0
        self._TwoFeetPoses['left'].orientation.z = 0.7074170719214404
        self._TwoFeetPoses['left'].orientation.w = 0

        self._TwoFeetPoses['right'].position.x = 0.25
        self._TwoFeetPoses['right'].position.y = -0.15
        self._TwoFeetPoses['right'].position.z = 0.09

        self._TwoFeetPoses['right'].orientation.x = 0
        self._TwoFeetPoses['right'].orientation.y = 0.7074162224043213
        self._TwoFeetPoses['right'].orientation.z = 0
        self._TwoFeetPoses['right'].orientation.w = 0.7067965984720317

        offset = Pose()
        offset.position.x = 0.3
        offset.position.y = 0
        offset.position.z = 0
        offset.orientation.x = 0
        offset.orientation.y = 0
        offset.orientation.z = 0
        offset.orientation.w = 1

        # get the offset relative to the robot in the world frame
        # below is for new models
        start_pose = getPoseFromRobotFrame(self._tf_listener, "/v1/Pelvis", offset)
        if start_pose == None:
            # unable to get relative pose (TF not available? sim not running?), just use the initial offset
            start_pose = offset

        start_pose.position.z = 0.0
        start_pose.orientation.x = 0
        start_pose.orientation.y = 0
        start_pose.orientation.z = 0
        start_pose.orientation.w = 1

        self.addWaypoint(None, start_pose, False)

    def tearDown(self):
        # publish an empty array to notify robot's path planning node
        array = MarkerArray()
        self._waypointPublisher.publish(array)
        self.clearPathMarkers()
        self.server.applyChanges()
        rospy.signal_shutdown("User deleted template.")

    def planFeetPathCb(self, feedback) :
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState(handle)
        if state == MenuHandler.CHECKED:
            print "Turning OFF footstep path determination"
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self._planningFootPath = False

            # delete feet
            for n in self._footstepInteractiveMarkers.keys() :
                self.removeInteractiveMarker(n)
                del self._footstepInteractiveMarkers[n]
                if n in self._footstepHeightControls.keys() :
                    del self._footstepHeightControls[n]

            self.menu_handler.setVisible(self._feetAdjustMenuID, True)
            self.menu_handler.setVisible(self._feetPublishMenuID, False)

        else:
            print "Turning ON footstep path determination"
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self._planningFootPath = True
            self.menu_handler.setVisible(self._feetAdjustMenuID, False)
            self.menu_handler.setVisible(self._feetPublishMenuID, True)
            self.menu_handler.setVisible(self._feetHeightMenuID, False)
            self.menu_handler.setVisible(self._syncToActualMenuID, False)
            self.menu_handler.setVisible(self._feetHeightMenuID, True)


        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def adjustFeet(self, feedback) :
        print "adding individual feet adjustment markers"
        self.menu_handler.setVisible(self._feetAdjustMenuID, True)
        self.menu_handler.setVisible(self._feetPublishMenuID, True)
        self.menu_handler.setVisible(self._feetHeightMenuID, True)
        self.menu_handler.setVisible(self._syncToActualMenuID, True)

        marker_data = MarkerArray()

        for i in range(2) :
            print "creating foot: ", str(i)
            footMarker = Marker()

            footMarker.header.frame_id = "/world"
            footMarker.header.seq = i
            footMarker.header.stamp = rospy.Time.now()

            if i==0 :
                footMarker.pose = self._TwoFeetPoses['right']
                footMarker.color.r = 1
                footMarker.color.g = 0
                footMarker.color.b = 1
                footMarker.color.a = 0.5
                footMarker.text = "right"
                footMarker.mesh_resource = self._rightFootMesh
            else :
                footMarker.pose = self._TwoFeetPoses['left']
                footMarker.color.r = 0
                footMarker.color.g = 1
                footMarker.color.b = 0
                footMarker.color.a = 0.5
                footMarker.mesh_resource = self._leftFootMesh
                footMarker.text = "left"

            footMarker.id = i
            footMarker.scale.x = 1
            footMarker.scale.y = 1
            footMarker.scale.z = 1
            footMarker.type = footMarker.MESH_RESOURCE

            marker_data.markers.append(footMarker)

        self.adjustTwoFeetCB(marker_data)

    def syncActualPoses(self, feedback) :
        print "syncing feet to actual"
        try:
            (trans,q) = self._tf_listener.lookupTransform('/world', '/v1/RightUpperFoot', rospy.Time(0))
            print "right"
            print trans, q

            self._TwoFeetPoses['right'].position.x = trans[0]
            self._TwoFeetPoses['right'].position.y = trans[1]
            self._TwoFeetPoses['right'].position.z = trans[2]

            self._TwoFeetPoses['right'].orientation.x = q[0]
            self._TwoFeetPoses['right'].orientation.y = q[1]
            self._TwoFeetPoses['right'].orientation.z = q[2]
            self._TwoFeetPoses['right'].orientation.w = q[3]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "failed to get right foot pose"


        try:
            (trans,q) = self._tf_listener.lookupTransform('/world', '/v1/LeftUpperFoot', rospy.Time(0))
            print "left"
            print trans, q

            self._TwoFeetPoses['left'].position.x = trans[0]
            self._TwoFeetPoses['left'].position.y = trans[1]
            self._TwoFeetPoses['left'].position.z = trans[2]

            self._TwoFeetPoses['left'].orientation.x = q[0]
            self._TwoFeetPoses['left'].orientation.y = q[1]
            self._TwoFeetPoses['left'].orientation.z = q[2]
            self._TwoFeetPoses['left'].orientation.w = q[3]


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "failed to get left foot pose"

        self.adjustFeet(feedback)

    def addWaypoint(self, parent_name=None, offset=Pose(), replace=False):

        # add a new waypoint if it's the first
        if len(self._waypoints) == 0:
            self.insertWaypointIntoTemplate(offset, replace)
            self.server.applyChanges()
            return True
        else:
            if parent_name:
                node = parent_name
            else:
                node = self._waypoints[0]

            index = self._waypoints.index(node)
            original_size = len(self._waypoints)

            # store any waypoints following the soon to be deleted waypoint
            if index < original_size:
                self.pushWaypointsAndResize(index)

            # insert waypoint into vector and server
            self.insertWaypointIntoTemplate(offset, replace)

            # pop any stored waypoints
            if self._wpStack:
                self.popWaypoints()

            if len(self._waypoints) > 1:
                self.createPaths()

            self.server.applyChanges()

            if len(self._waypoints) > original_size:
                return True
            else:
                return False

    def deleteWaypoint(self, name):
        index = self._waypoints.index(name)
        original_size = len(self._waypoints)

        # store waypoints
        if index < original_size:
            self.pushWaypointsAndResize(index)

        # delete from vector and server
        if (self.removeWaypointFromTemplate(name)):
            # pop any stored waypoints
            if self._wpStack:
                self.popWaypoints()

            if len(self._waypoints) > 1:
                self.createPaths()

            self.server.applyChanges()

        if len(self._waypoints) == original_size - 1:
            return True
        else:
            return False

    def getNextID(self):
        id = len(self._waypoints)
        if id < 10:
            str_id = "0" + str(id)
        else:
            str_id = str(id)
        return str_id

    def insertWaypointIntoTemplate(self, offset, replace):

        key = "/operator/Waypoint/" + str(self.id) + "/waypoint/" + self.getNextID()

        # init interactivemarker waypoint and adjust offset
        waypoint = InteractiveMarker()
        waypoint.header.frame_id = self._frame_id
        waypoint.name = key
        waypoint.description = str(self.id) + " - step" + self.getNextID()
        waypoint.scale = 0.25
        waypoint.pose = offset

        # unless we're replacing a waypoint, slightly adjust the position so we're not inserting
        # directly on top of an existing waypoint
        if not replace:
            waypoint.pose.position.y += 0.1
            # slightly raise z to allow easier selection in rviz
            if waypoint.pose.position.z <= 0.01:
                waypoint.pose.position.z = 0.01

        # create a control to translate in XY plane and add arrow visualization.
        # also add rotate Z control
        translate = CreateTransRotControl("TranslateXY")
        translate.markers.append(self.createArrow(len(self._waypoints)))
        waypoint.controls.append(translate)
        waypoint.controls.append(CreateTransRotControl("RotateZ"))

        radius = self.createCylinder(len(self._waypoints))
        radius_control = CreateVisualControlFromMarker(radius)
        waypoint.controls.append(radius_control)

        self.addInteractiveMarker(waypoint)
        self.attachMenuHandler(waypoint)
        self._waypoints.append(waypoint.name)

        # adjust path callback
        self.server.setCallback(waypoint.name, self.adjustPath)

    def addWaypointCb(self, feedback):
        self.addWaypoint(feedback.marker_name, feedback.pose)

    def footstepInCB(self, data) :
        if self._planningFootPath :
            self._markerData = data
            self.translateFeetToInteractiveMarkers()

    def adjustTwoFeetCB(self, data) :
        self._markerData = data
        self.translateFeetToInteractiveMarkers()

    def translateFeetToInteractiveMarkers(self) :
        # if not self._planningFootPath: return

        marker_names = []

        for m in self._markerData.markers :
            if str(m.id) in self._footstepInteractiveMarkers :
                self.server.setPose(str(m.id), m.pose)
                #print "not adding footstep: ", str(m.id)
                self.removeInteractiveMarker(str(m.id))

            #else :
            im = InteractiveMarker()
            im.header.frame_id = deepcopy(m.header.frame_id)
            im.header.seq = m.header.seq
            im.header.stamp = m.header.stamp

            im.name = str(m.id)
            im.pose = m.pose
            im.scale = 0.25
            translate = CreateTransRotControl("TranslateXZ")
            m.scale.x = 1
            m.scale.y = 1
            m.scale.z = 1

            if m.id%2 == 1:
                m.mesh_resource = self._leftFootMesh
            else :
                m.mesh_resource = self._rightFootMesh

            m.type = m.MESH_RESOURCE

            # Rorig = kdl.Rotation.Quaternion(m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w)
            # Roff = kdl.Rotation.RPY(0, 1.57, 3.14)
            # R = Rorig*Roff
            # q = R.GetQuaternion()
            # m.pose.position.x -= 0.06
            # m.pose.position.y += 0.0
            # m.pose.position.z += 0.09
            # m.pose.orientation.x = q[0]
            # m.pose.orientation.y = q[1]
            # m.pose.orientation.z = q[2]
            # m.pose.orientation.w = q[3]
            translate.markers.append(m)
            im.controls.append(translate)
            im.controls.append(CreateTransRotControl("RotateX"))


            if str(m.id) in self._footstepHeightControls.keys() :
                if self._footstepHeightControls[str(m.id)] :
                    print "adding height controls to ", str(m.id)
                    im.controls.append(CreateTransRotControl("TranslateX"))

            self._footstepInteractiveMarkers[im.name] = deepcopy(im)

            self.addInteractiveMarker(self._footstepInteractiveMarkers[im.name], self.processFootstepFeedback)
            self.attachMenuHandler(self._footstepInteractiveMarkers[im.name])

            marker_names.append(str(m.id))

        for n in self._footstepInteractiveMarkers.keys() :
            #print "checking ", n, " in keys"
            if n not in marker_names :
                self.removeInteractiveMarker(n)
                del self._footstepInteractiveMarkers[n]
                if n in self._footstepHeightControls.keys() :
                    del self._footstepHeightControls[n]

        self.server.applyChanges()


    def processFootstepFeedback(self, feedback) :
        #if not self._planningFootPath: return

        if (feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP):
            print "in processFootstepFeedback: "
            print feedback

            sid = str(feedback.marker_name)
            pose = feedback.pose

            if self._planningFootPath :
                print "sid: ", sid
                for m in self._markerData.markers:
                    if str(m.id) == sid:
                        m.pose = pose
            else :
                if not self._planningFootPath:
                    if sid == "0":
                        self._TwoFeetPoses['right'] = pose
                        print "setting right foot adjustment pose"
                    if sid == "1":
                        self._TwoFeetPoses['left'] = pose
                        print "setting left foot adjustment pose"
                    self.adjustFeet(feedback)

    def publishWaypoints(self, feedback):
        array = MarkerArray()
        count = 0
        for waypoint_name in self._waypoints:
            waypointIM = self.server.get(waypoint_name)
            # waypoint = self.createArrow(count)
            waypoint = self.createCylinder(count)
            waypoint.header = waypointIM.header
            waypoint.pose = waypointIM.pose
            waypoint.ns = waypointIM.name
            marker = GetVisualMarker(waypointIM)
            waypoint.scale = marker.scale
            array.markers.append(waypoint)

        if (feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP):
            self._waypointPublisher.publish(array)

    def publishFootseps(self, feedback) :
        array = MarkerArray()
        count = 0
        for footstep in self._footstepInteractiveMarkers.keys():
            marker = self._footstepInteractiveMarkers[footstep].controls[0].markers[0]
            array.markers.append(marker)
        self._footstepPublisher.publish(array)


    def pushWaypointsAndResize(self, index):

        # TODO: The vector of waypoints helps keep track of added waypoints in the template. It looks as
        # if only the "name" field of the waypoint (InteractiveMarker) is getting used. If this is the
        # case, the vector can be a vector of strings containing waypoint names, instead of actual
        # InteractiveMarker objects.
        # Get name of waypoint from vector, and use that to get the InteractiveMarker on the server.
        for i in range(len(self._waypoints)-1, index, -1):
            wpName = self._waypoints[i]
            interactive_marker = self.server.get(wpName)
            # push a copy of the marker onto the stack and delete from the server
            self._wpStack.append(interactive_marker)
            self.server.erase(wpName)

        # // Now resize vector to the index.
        # // Because our vector of waypoints are both numbered sequentially and don't skip numbers,
        # // We can just resize the vector to the index.
        # // Example:
        # // for vector {0, 1, 2, 3, 4, 5 ,6 ,7 ,8, 9}
        # // we delete index 5, and pop remaining into a stack
        # //   stack {9, 8, 7, 6} <- top
        # // we want the new vector to be
        # //   vector {0, 1, 2, 3, 4}
        # // So we can resize it using the index. This seems dirty. Is this dirty?
        self._waypoints = self._waypoints[:index+1]

    def popWaypoints(self):
        # // Now pop each InteractiveMarker from the stack. Grab the pose from the InteractiveMarker
        # // and call addWaypoint(), passing in the offset to create a new marker. The addWaypoint()
        # // method will handle naming of the waypoints correctly, and will also add the waypoint to
        # // the template vector for tracking.
        while self._wpStack:
            waypoint = self._wpStack.pop()
            offset = waypoint.pose
            self.insertWaypointIntoTemplate(offset, True)

    def deleteWaypointCb(self, feedback):
        self.deleteWaypoint(feedback.marker_name)

    def removeWaypointFromTemplate(self, name):
        count = len(self._waypoints)
        if name in self._waypoints:
            self._waypoints.remove(name)

        self.removeInteractiveMarker(name)

        # return true if a waypoint was deleted
        if len(self._waypoints) < count:
            return self.server.erase(name)
        else:
            return False

    def createArrow(self, id):
        arrow = CreatePrimitiveMarker(0.0, Marker.ARROW, id)
        arrow.color = self._color
        # groovy changed how scaling affects interactive markers
        arrow.scale.x = 0.35 * scaleFactorX
        arrow.scale.y = 0.05 * scaleFactorY
        arrow.scale.z = 0.05 * scaleFactorZ
        return arrow

    def createCylinder(self, id):
        radius = CreatePrimitiveMarker(0.0, Marker.CYLINDER, id)
        radius.color.r = 0.5
        radius.color.a = 0.5
        radius.scale.x = 0.25
        radius.scale.y = 0.25
        radius.scale.z = 0.01
        radius.pose.position.z = -0.01
        return radius

    def pathCb(self, feedback):
        pose = Pose()
        pose.position = feedback.mouse_point
        # TODO: it's late
        # TODO: midpoint in util.CreatePath is namespacing it with PathMidpoint
        # TODO: we want to add a waypoint using the path's parent, which is not the
        # TODO: name with PathMidpoint. fiiiiiiiix
        key = feedback.control_name.rsplit('PathMidpoint',1)[0]
        self.addWaypoint(key, pose)


    def addHeightControl(self, feedback) :
        print "MENU FEEDBACK:"
        print feedback

        sid = str(feedback.marker_name)
        if sid in self._footstepHeightControls :
            self._footstepHeightControls[sid] = not self._footstepHeightControls[sid]
        else :
            self._footstepHeightControls[sid] = True

        if self._planningFootPath :
            self.footstepInCB(self._markerData)
        else :
            self.adjustTwoFeetCB(self._markerData)

    def addPathMarker(self, name):
        self._pathMarkers.append(name)

    def clearPathMarkers(self):
        for marker in self._pathMarkers:
            self.server.erase(marker)
        self._pathMarkers = []

    def adjustPath(self, feedback):
        #if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        index = self._waypoints.index(feedback.marker_name)

        if len(self._waypoints) > 1:
            if index == 0:
                self.createWaypointPath(index, index+1)
            elif index == len(self._waypoints) - 1:
                self.createWaypointPath(index-1, index)
            else:
                self.createWaypointPath(index-1, index)
                self.createWaypointPath(index, index+1)

        self.publishWaypoints(feedback)
        self.server.applyChanges()

    def createWaypointPath(self, parent_index, child_index):
        parent_name = self._waypoints[parent_index]
        child_name = self._waypoints[child_index]
        path = CreatePath(self.server, self._frame_id, parent_name, child_name)
        self.addPathMarker(path.name)
        self._pathHandler.apply(self.server, path.name)

    def createPaths(self):
        self.clearPathMarkers()
        if len(self._waypoints) > 1:
            for i in range(len(self._waypoints)-1):
                self.createWaypointPath(i, i+1)

    def dynamicReconfigureCb(self, config, level):
        if self.server:
            # change frame_ids
            if self._frame_id != config.frame_id:
                self._frame_id = config.frame_id
                UpdateFrameIDs(self, self._frame_id)

            # change publishers
            if self._waypointPublishTopic != config.waypoint_publish_topic:
                self._waypointPublishTopic = config.waypoint_publish_topic
                self._waypointPublisher = rospy.Publisher(self._waypointPublishTopic, MarkerArray)

            if self._footstepPublishTopic != config.footstep_publish_topic:
                self._footstepPublishTopic = config.footstep_publish_topic
                self._footstepPublisher = rospy.Publisher(self._footstepPublishTopic, MarkerArray)

            # change subscribers
            if self._footstepSubscribeTopic != config.footstep_subscribe_topic:
                self._footstepSubscribeTopic = config.footstep_subscribe_topic
                rospy.Subscriber(self._footstepSubscribeTopic, MarkerArray, self.footstepInCB)

            index = config.waypoint
            if index < 10:
                index = '0' + str(index)
            else:
                index = str(index)

            if self._height != config.height:
                self._height = config.height
                if self._markerData != None:
                    if self._planningFootPath :
                        self.footstepInCB(self._markerData)
                    else :
                        self.adjustTwoFeetCB(self._markerData)

            # find the waypoint in the interactive marker map
            waypoint = None
            for name, marker in self.marker_map.iteritems():
                if name.endswith(index):
                    waypoint = marker
                    break

            if waypoint:
                # iterate over controls to find the "radius" control
                # TODO: create a util function to do this
                for control in waypoint.controls:
                    if control.name == 'visual':
                        # remove old control, update its radius, insert it back into the control
                        new_control = control
                        waypoint.controls.remove(control)
                        new_control.markers[0].scale.x = config.radius
                        new_control.markers[0].scale.y = config.radius
                        waypoint.controls.append(new_control)
                        # inserted the updated waypoint marker into the server (overwriting old)
                        # and apply changes
                        self.server.insert(waypoint)
                        self.server.applyChanges()
        return config

if __name__ == '__main__':
    count = '0'
    # backwards compatibility, pass in True for second argument to force
    # each template node to publish to a different topic
    separate_topic = False
    if len(sys.argv) > 1:
        count = sys.argv[1]
        separate_topic = False
    if len(sys.argv) > 2:
        count = sys.argv[1]
        import ast
        separate_topic = ast.literal_eval(sys.argv[2])
    key = "WaypointTemplate" + count
    rospy.init_node(key)
    if separate_topic:
        server = InteractiveMarkerServer(key)
    else:
        server = InteractiveMarkerServer("MarkerTemplates", key)
    template = Waypoint(server, key, int(count))
    while not rospy.is_shutdown():
        rospy.spin()