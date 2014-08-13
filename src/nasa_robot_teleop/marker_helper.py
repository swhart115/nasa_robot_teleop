#!/usr/bin/env python

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

def makeEmptyMarker( dummyBox=True ):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker

def makeBox( msg ):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.25
    marker.scale.y = msg.scale * 0.25
    marker.scale.z = msg.scale * 0.25
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker

def makeSphere( msg, scale ):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * scale
    marker.scale.y = msg.scale * scale
    marker.scale.z = msg.scale * scale
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    return marker

def makeMesh( msg, mesh_str, p, sf=1, alpha=1, mesh_use_embedded_materials=True ):
    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    marker.scale.x = sf
    marker.scale.y = sf
    marker.scale.z = sf
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = alpha
    marker.pose = p
    marker.mesh_resource = mesh_str
    marker.mesh_use_embedded_materials = mesh_use_embedded_materials
    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = False
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def makeBoxMenu( msg ):
    control =  InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.always_visible = False
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def makeMeshControl( msg, mesh_name, p ):
    control =  InteractiveMarkerControl()
    control.always_visible = False
    control.markers.append( makeMesh(msg, mesh_name, p, 0.99) )
    msg.controls.append( control )
    return control

def makeSphereControl( msg, sphere ):
    control =  InteractiveMarkerControl()
    control.always_visible = False
    control.markers.append( makeSphere(msg, sphere) )
    msg.controls.append( control )
    return control

def makeXTransControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return control

def makeYTransControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return control

def makeZTransControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    return control

def makeXRotControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    return control

def makeYRotControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    return control

def makeZRotControl () :
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    return control

def make6DOFControls() :
    controls = []
    controls.append(makeXTransControl())
    controls.append(makeYTransControl())
    controls.append(makeZTransControl())
    controls.append(makeXRotControl())
    controls.append(makeYRotControl())
    controls.append(makeZRotControl())
    return controls
