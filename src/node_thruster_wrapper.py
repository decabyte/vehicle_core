#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('vehicle_core')

from vehicle_interface.msg import ThrusterCommand, ThrusterFeedback
from vehicle_interface.srv import DictionaryService, DictionaryServiceResponse
from diagnostic_msgs.msg import KeyValue

# topics
TOPIC_IN_REQ = 'thrusters/request'
TOPIC_OUT_REQ = 'thrusters/commands'
TOPIC_IN_FEED = 'thrusters/status'
TOPIC_OUT_FEED = 'thrusters/feedback'
SRV_FAULT = 'thrusters/faults'

# node status (faults)
FAULT_NONE = 'none'
FAULT_HARD_LIM = 'hard_limit'
FAULT_THRT_LOSS = 'thrust_loss'


class ThrusterWrapper(object):

    def __init__(self, name, req_in, req_out, feed_in, feed_out):
        self.name = name
        self.req_input = req_in
        self.req_output = req_out
        self.feed_input = feed_in
        self.feed_output = feed_output

        self.fault_type = FAULT_NONE
        self.th_min = np.ones(6) * -100
        self.th_max = np.ones(6) * 100

        # ros interface
        self.sub_request = rospy.Subscriber(self.req_input, ThrusterCommand, self.handle_input, tcp_nodelay=True, queue_size=10)
        self.sub_feedback = rospy.Subscriber(self.feed_input, ThrusterFeedback, self.handle_feedback, tcp_nodelay=True, queue_size=10)

        self.pub_request = rospy.Publisher(self.req_output, ThrusterCommand, tcp_nodelay=True, queue_size=10)
        self.pub_feedback = rospy.Publisher(self.feed_output, ThrusterFeedback, tcp_nodelay=True, queue_size=10)

        self.srv_fault = rospy.Service(SRV_FAULT, DictionaryService, self.handle_fault)



    def handle_feedback(self, data):
        # TODO: not implemented yet, just pass-through the feedback.
        self.pub_feedback.publish(data)


    def handle_input(self, data):
        # read the input command
        th_in = np.array(data.throttle[0:6])
        th_out = np.array(data.throttle[0:6])

        if self.fault_type == FAULT_HARD_LIM:
            # fault option 2
            th_out = np.clip(th_in, self.th_min, self.th_max)
        elif self.fault_type == FAULT_THRT_LOSS:
            # fault option 3
            pass
        else:
            # pass-through option
            pass

        # resend command
        tc = ThrusterCommand()
        tc.header.stamp = rospy.Time.now()
        tc.throttle = th_out
        self.pub_request.publish(tc)


    def handle_fault(self, req):
        # parse received dictionary
        params = dict()
        for pair in req.request:
            params[pair.key] = pair.value

        # select fault type
        ftype = params.get('fault_type', FAULT_NONE)

        if ftype == FAULT_HARD_LIM:
            try:
                params['th_min'] = params['th_min'].replace('[', '').replace(']', '')
                params['th_max'] = params['th_max'].replace('[', '').replace(']', '')
                tmin = np.fromstring(params['th_min'], sep=',')
                tmax = np.fromstring(params['th_max'], sep=',')

                # check if the input is correct
                if len(tmin) != 6:
                    tmin = np.ones(6) * -100

                if len(tmax) != 6:
                    tmax = np.ones(6) * 100

                self.th_min = np.clip(tmin, -100, 0)
                self.th_max = np.clip(tmax, 0, 100)

                self.fault_type = FAULT_HARD_LIM
                rospy.logwarn('%s: hard limit fault enabled', self.name)
            except Exception as e:
                rospy.logerr('%s: bad service call: %s', self.name, e)
        else:
            self.fault_type = FAULT_NONE
            self.th_max = np.ones(6) * 100
            self.th_min = np.ones(6) * -100
            rospy.logwarn('%s: resetting faults', self.name)


        res = list()
        res.append(KeyValue('fault_type', str(self.fault_type)))
        res.append(KeyValue('th_min', str(self.th_min)))
        res.append(KeyValue('th_max', str(self.th_max)))

        return DictionaryServiceResponse(result=True, response=res)



if __name__ == '__main__':
    rospy.init_node('thrusters_wrapper')
    name = rospy.get_name()

    # load configuration
    req_input = rospy.get_param('~req_input', TOPIC_IN_REQ)
    req_output = rospy.get_param('~req_output', TOPIC_OUT_REQ)
    feed_input = rospy.get_param('~feed_input', TOPIC_IN_FEED)
    feed_output = rospy.get_param('~feed_output', TOPIC_OUT_FEED)

    # print configuration
    rospy.loginfo('%s: fault injector init ... ', name)
    rospy.loginfo('%s: request input topic: %s', name, req_input)
    rospy.loginfo('%s: request output topic: %s', name, req_output)
    rospy.loginfo('%s: feedback input topic: %s', name, feed_input)
    rospy.loginfo('%s: feedback output topic: %s', name, feed_output)

    tw = ThrusterWrapper(name, req_input, req_output, feed_input, feed_output)
    rospy.spin()
