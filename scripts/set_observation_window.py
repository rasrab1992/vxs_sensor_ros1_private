#!/usr/bin/env python3
import rospy
from vxs_sensor_ros1.srv import UpdateObservationWindow

if __name__ == "__main__":
    rospy.init_node("set_observation_window_client")
    # Params (override them from the launch file)
    on_time     = rospy.get_param("~on_time", 3)
    period_time = rospy.get_param("~period_time", 10)

    # Wait for the service exposed by the node
    service_name = "/vxs_node/update_observation_window"  # adjust if you rename the node
    rospy.loginfo("Waiting for %s ...", service_name)
    rospy.wait_for_service(service_name)

    try:
        call = rospy.ServiceProxy(service_name, UpdateObservationWindow)
        resp = call(on_time, period_time)
        level = rospy.loginfo if resp.success else rospy.logwarn
        level("UpdateObservationWindow -> success=%s, msg=%s", resp.success, resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
