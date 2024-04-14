#!/usr/bin/python

import rospy
import dronecan
from sensor_msgs.msg import BatteryState

class RosDronecan():
    def __init__(self):
        rospy.loginfo("Setting Up the dronecan bridge...")
        self.can_interface = rospy.get_param('~can_interface', 'can0')
        self.node_id = rospy.get_param('~node_id', 127)
        self.bitrate = rospy.get_param('~bitrate', 1000000)
        self.enable_nodeid_server = rospy.get_param('~enable_nodeid_server', False)
        self.current_offset = rospy.get_param('~current_offset', 0.0)
        self.voltage_offset = rospy.get_param('~voltage_offset', 0.0)
        self.calculate_percentage = rospy.get_param('~calculate_percentage', False)
        self.cell_empty = rospy.get_param('~cell_empty', 3.3)
        self.cell_full = rospy.get_param('~cell_full', 4.2)
        self.cell_num = rospy.get_param('~cell_num', 6)
        self.negative_charge = rospy.get_param('~calculate_percentage', True)
        self.report_temperature = rospy.get_param('~report_temperature', False)
        self.battery_empty = self.cell_empty * self.cell_num
        self.battery_full = self.cell_full * self.cell_num
        self.node_monitor = None
        self.dynamic_node_id_allocator = None
    
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
        if self.enable_nodeid_server:
            self.node_monitor = dronecan.app.node_monitor.NodeMonitor(self.node)
            self.dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(self.node, self.node_monitor)
        self.node.mode = dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL
        self.node.health = dronecan.uavcan.protocol.NodeStatus().HEALTH_OK
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
        battery_status.voltage = event.message.voltage + self.voltage_offset
        battery_status.current = event.message.current + self.current_offset
        if self.report_temperature:
            battery_status.temperature = event.message.temperature - 273.15
        else:
            battery_status.temperature = 0.0
        if battery_status.current > 0:
            if self.negative_charge:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            else:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            if self.negative_charge:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if self.calculate_percentage:
            battery_status.percentage = (battery_status.voltage - self.battery_empty) / (self.battery_full - self.battery_empty) * 100
            if battery_status.percentage > 100:
                battery_status.percentage = 100.0
            if battery_status.percentage < 0:
                battery_status.percentage = 0.0
        else:
            battery_status.percentage = event.message.state_of_charge_pct
        self._ros_pub_battery_state.publish(battery_status)

if __name__ == "__main__":
    rospy.init_node('rosdronecan')
    bridge = RosDronecan()
    bridge.run()
