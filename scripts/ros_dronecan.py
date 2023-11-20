#!/usr/bin/python

import rospy
import dronecan
from sensor_msgs.msg import BatteryState

class RosDronecan():
    def __init__(self, can_interface="can0", node_id=127, bitrate=1000000):
        self.can_interface = can_interface
        self.node_id = node_id
        self.bitrate = bitrate
        rospy.loginfo("Setting Up the dronecan bridge...")
    
    def run(self):
        rospy.loginfo("Starting dronecan bridge on interface %s", self.can_interface)
        self._ros_pub_battery_state = rospy.Publisher('/battery_state', BatteryState, queue_size=5)
        node_info = dronecan.uavcan.protocol.GetNodeInfo.Response()
        node_info.name = "org.dronecan.rosdronecan"
        node_info.software_version.major = 0
        node_info.software_version.minor = 1
        node_info.hardware_version.unique_id = b'12345'
        self.node = dronecan.make_node(self.can_interface, node_id=self.node_id, bitrate=self.bitrate, node_info=node_info)
        self.node.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, self.node_battery_status_callback)
        while not rospy.is_shutdown():
            try:
                self.node.spin(1)
            except dronecan.transport.TransferError:
                pass
                # rospy.logerr("dronecan transfer error")
        self.node.close()
        rospy.loginfo("Dronecan gateway closed")
    
    def node_battery_status_callback(self, event):
        # rospy.loginfo(dronecan.to_yaml(event))
        battery_status = BatteryState()
        battery_status.voltage = event.message.voltage
        battery_status.current = event.message.current
        battery_status.percentage = event.message.state_of_charge_pct
        battery_status.temperature = event.message.temperature - 273.15
        battery_status.power_supply_status = 2
        self._ros_pub_battery_state.publish(battery_status)

if __name__ == "__main__":
    rospy.init_node('rosdronecan')
    bridge = RosDronecan(can_interface=rospy.get_param('~can_interface', "can0"),
                         node_id=rospy.get_param('~node_id', 127),
                         bitrate=rospy.get_param('~bitrate', 1000000))
    bridge.run()

