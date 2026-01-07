#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import actionlib
import rospy
from std_msgs.msg import String
import rospy
import threading
import moveit_commander
from rois_env.msg import *
from rois_env.srv import *
from rois_env.srv import MoveCommunication

class ComponentBase:
    def __init__(self, robotname, comp_ref):
        self.robotname = robotname
        self.comp_ref = comp_ref

        # 状態
        self.comp_state = "UNINITIALIZED"
        self.state = "idle"
        rospy.loginfo(f'Component {self.comp_ref} status: {self.comp_state}')

        # get_state service
        state_name = self.robotname + '/get_state/' + self.comp_ref
        self.state_service = rospy.Service(state_name, component_status, self._handle_get_state)
        rospy.loginfo("component status server is ready")

        # execute action
        exe_name = self.robotname + '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, executeAction, self._execute_dispatcher, False)
        self.server.start()
        rospy.loginfo(f"Action server started: {exe_name}")

        # completed publisher
        self.pub = rospy.Publisher(self.robotname + '/completed_command', completed, queue_size=1, latch=True)

        # READYへ
        self.comp_state = "READY"
        rospy.loginfo(f'Component {self.comp_ref} status: {self.comp_state}')

    # ----------------------------
    # Action dispatcher
    # ----------------------------
    def _execute_dispatcher(self, goal):
        self.comp_state = "BUSY"
        rospy.loginfo(f'Component {self.comp_ref} status: {self.comp_state}')

        command = goal.command_name
        rospy.loginfo(f"Received command: {command}")

        self.feedback = executeFeedback()
        self.result = executeResult()

        try:
            if command == "start":
                self.start()
            elif command == "stop":
                self.stop()
            elif command == "suspend":
                self.suspend()
            elif command == "resume":
                self.resume()
            else:
                raise Exception("Unknown command")

        except Exception as e:
            rospy.logerr(f"Execution error: {e}")
            self.comp_state = "ERROR"
            self.result.success = "False"
            self.server.set_aborted(self.result)

    # ----------------------------
    # Status service
    # ----------------------------
    def _handle_get_state(self, req):
        if req.component_name == self.comp_ref:
            rospy.loginfo(f"Current state requested: {self.comp_state}")
            return component_statusResponse(self.comp_state)
        else:
            return component_statusResponse("UNKNOWN")

    # ----------------------------
    # 共通完了通知
    # ----------------------------
    def notify_completed(self, status):
        msg = completed()
        msg.command_id = self.comp_ref
        msg.status = status
        self.pub.publish(msg)

        self.comp_state = "READY"
        rospy.loginfo(f'Component {self.comp_ref} status: {self.comp_state}')

    # ----------------------------
    # 以下は派生クラスで実装必須
    # ----------------------------
    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def suspend(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError
