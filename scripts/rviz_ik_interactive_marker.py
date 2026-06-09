#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker


def quaternionFromRpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class RvizIkMarkerNode:
    def __init__(self) -> None:
        self.frameId_ = rospy.get_param("~frame_id", "world")
        self.serverNs_ = rospy.get_param("~server_ns", "rkv_ik_marker")
        self.markerName_ = rospy.get_param("~marker_name", "ik_target")
        self.publishTopic_ = rospy.get_param("~publish_topic", "/rkv/ik_target_pose")
        self.publishRate_ = float(rospy.get_param("~publish_rate_hz", 30.0))
        self.feedbackPublishRate_ = float(rospy.get_param("~feedback_publish_rate_hz", self.publishRate_))
        self.markerScale_ = float(rospy.get_param("~marker_scale", 0.30))
        self.alwaysPublish_ = bool(rospy.get_param("~always_publish", False))
        self.latch_ = bool(rospy.get_param("~latch", True))
        self.positionOnlyHint_ = bool(rospy.get_param("~position_only_hint", True))

        initX = float(rospy.get_param("~init_x", 0.45))
        initY = float(rospy.get_param("~init_y", 0.0))
        initZ = float(rospy.get_param("~init_z", 1.0))
        initRoll = math.radians(float(rospy.get_param("~init_roll_deg", 0.0)))
        initPitch = math.radians(float(rospy.get_param("~init_pitch_deg", 0.0)))
        initYaw = math.radians(float(rospy.get_param("~init_yaw_deg", 0.0)))

        self.pub_ = rospy.Publisher(self.publishTopic_, PoseStamped, queue_size=5, latch=self.latch_)
        self.server_ = InteractiveMarkerServer(self.serverNs_)
        self.poseLock_ = threading.Lock()
        self.lastFeedbackPublishSec_ = 0.0
        self.latestPose_ = Pose()
        self.latestPose_.position.x = initX
        self.latestPose_.position.y = initY
        self.latestPose_.position.z = initZ
        self.latestPose_.orientation = quaternionFromRpy(initRoll, initPitch, initYaw)

        self.makeMarker_()
        self.server_.applyChanges()

        period = 1.0 / max(self.publishRate_, 1.0)
        rospy.Timer(rospy.Duration.from_sec(period), self.onTimer_)
        self.publishPose_(self.latestPose_, rospy.Time.now())

        rospy.loginfo("Interactive marker server started: ns=%s frame=%s", self.serverNs_, self.frameId_)
        rospy.loginfo("Publishing IK target pose to: %s", self.publishTopic_)
        rospy.loginfo("Feedback publish rate: %.1f Hz, always_publish=%s", self.feedbackPublishRate_, self.alwaysPublish_)

    def makeAxisControl_(self, name: str, x: float, y: float, z: float, mode: int) -> InteractiveMarkerControl:
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = 1.0
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        control.interaction_mode = mode
        return control

    def makeMarker_(self) -> None:
        marker = InteractiveMarker()
        marker.header.frame_id = self.frameId_
        marker.name = self.markerName_
        marker.description = "WBC Chain IK Target (position priority)" if self.positionOnlyHint_ else "WBC Chain IK Target (pose)"
        marker.scale = self.markerScale_
        marker.pose = self.latestPose_

        visCtrl = InteractiveMarkerControl()
        visCtrl.name = "visual"
        visCtrl.always_visible = True
        visCtrl.interaction_mode = InteractiveMarkerControl.NONE

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = marker.scale * 0.22
        cube.scale.y = marker.scale * 0.22
        cube.scale.z = marker.scale * 0.22
        cube.color.r = 0.05
        cube.color.g = 0.75
        cube.color.b = 0.95
        cube.color.a = 0.90
        visCtrl.markers.append(cube)
        marker.controls.append(visCtrl)

        marker.controls.append(self.makeAxisControl_("move_x", 1.0, 0.0, 0.0, InteractiveMarkerControl.MOVE_AXIS))
        marker.controls.append(self.makeAxisControl_("rotate_x", 1.0, 0.0, 0.0, InteractiveMarkerControl.ROTATE_AXIS))
        marker.controls.append(self.makeAxisControl_("move_y", 0.0, 1.0, 0.0, InteractiveMarkerControl.MOVE_AXIS))
        marker.controls.append(self.makeAxisControl_("rotate_y", 0.0, 1.0, 0.0, InteractiveMarkerControl.ROTATE_AXIS))
        marker.controls.append(self.makeAxisControl_("move_z", 0.0, 0.0, 1.0, InteractiveMarkerControl.MOVE_AXIS))
        marker.controls.append(self.makeAxisControl_("rotate_z", 0.0, 0.0, 1.0, InteractiveMarkerControl.ROTATE_AXIS))

        free3d = InteractiveMarkerControl()
        free3d.name = "move_rotate_3d"
        free3d.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        marker.controls.append(free3d)

        self.server_.insert(marker, self.processFeedback_)

    def publishPose_(self, pose: Pose, stamp: rospy.Time) -> None:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frameId_
        msg.pose = pose
        self.pub_.publish(msg)

    def copyPose_(self, pose: Pose) -> Pose:
        poseCopy = Pose()
        poseCopy.position.x = pose.position.x
        poseCopy.position.y = pose.position.y
        poseCopy.position.z = pose.position.z
        poseCopy.orientation.x = pose.orientation.x
        poseCopy.orientation.y = pose.orientation.y
        poseCopy.orientation.z = pose.orientation.z
        poseCopy.orientation.w = pose.orientation.w
        norm = math.sqrt(
            poseCopy.orientation.x * poseCopy.orientation.x
            + poseCopy.orientation.y * poseCopy.orientation.y
            + poseCopy.orientation.z * poseCopy.orientation.z
            + poseCopy.orientation.w * poseCopy.orientation.w
        )
        if norm > 1e-9:
            poseCopy.orientation.x /= norm
            poseCopy.orientation.y /= norm
            poseCopy.orientation.z /= norm
            poseCopy.orientation.w /= norm
        else:
            poseCopy.orientation.w = 1.0
        return poseCopy

    def processFeedback_(self, feedback: InteractiveMarkerFeedback) -> None:
        event = feedback.event_type
        if event not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
            InteractiveMarkerFeedback.MOUSE_DOWN,
        ):
            return

        nowSec = rospy.Time.now().to_sec()
        if event == InteractiveMarkerFeedback.POSE_UPDATE and self.feedbackPublishRate_ > 0.0:
            minPeriod = 1.0 / self.feedbackPublishRate_
            if nowSec - self.lastFeedbackPublishSec_ < minPeriod:
                return
            self.lastFeedbackPublishSec_ = nowSec

        with self.poseLock_:
            self.latestPose_ = feedback.pose
            poseCopy = self.copyPose_(self.latestPose_)

        self.publishPose_(poseCopy, feedback.header.stamp if feedback.header.stamp != rospy.Time() else rospy.Time.now())

    def onTimer_(self, _event) -> None:
        if not self.alwaysPublish_:
            return
        with self.poseLock_:
            pose = self.copyPose_(self.latestPose_)
        self.publishPose_(pose, rospy.Time.now())


def main() -> None:
    rospy.init_node("rviz_ik_interactive_marker")
    RvizIkMarkerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
