#!/usr/bin/env python3

import rospy
import rospkg
import dronecan
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import ColorRGBA
from rover6_dronecan.srv import SetLED, SetLEDRequest, SetLEDResponse, SetLid, SetLidRequest, SetLidResponse
from rover6_dronecan.msg import ControllerStatus
from threading import Thread
from time import sleep

class Rover6Dronecan():
    def __init__(self):
        rospy.loginfo("Setting Up the rover6 dronecan bridge...")
        self.can_interface = rospy.get_param('~can_interface', 'can0')
        self.node_id = rospy.get_param('~node_id', 127)
        self.controller_node_id = rospy.get_param('~controller_node_id', 97)
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
        rospy.loginfo("Starting rover6 dronecan bridge on interface %s", self.can_interface)
        self._ros_pub_battery_state = rospy.Publisher('/battery_state', BatteryState, queue_size=5)
        self._ros_pub_controller_status = rospy.Publisher('/rover6_controller_status', ControllerStatus, queue_size=5)
        self._ros_set_lid_service = rospy.Service('/rover6/lid/set_lid', SetLid, self.set_lid_service)
        self._ros_set_knock_service = rospy.Service('/rover6/lid/set_knock', SetBool, self.set_knock_service)
        self._ros_set_led_service = rospy.Service('/rover6/set_led', SetLED, self.set_led_service)
        self.node_info = dronecan.uavcan.protocol.GetNodeInfo.Response()
        self.node_info.name = "org.dronecan.rosdronecan"
        self.node_info.software_version.major = 0
        self.node_info.software_version.minor = 1
        self.node_info.hardware_version.unique_id = b'12345'
        self.dronecan_node = dronecan.make_node(self.can_interface, node_id=self.node_id, bitrate=self.bitrate,
                                                node_info=self.node_info, mode=dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL)
        self.node_monitor = dronecan.app.node_monitor.NodeMonitor(self.dronecan_node)
        if self.enable_nodeid_server:
            rospy.loginfo("Starting dronecan dynamic id allocator")
            self.dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(self.dronecan_node, self.node_monitor)
        self.dronecan_node.health = dronecan.uavcan.protocol.NodeStatus().HEALTH_OK
        self.dronecan_node.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, self.node_battery_status_callback)
        self.dronecan_node.add_handler(dronecan.com.zxdynamics.controller.ControllerStatus, self.rover6_controller_status_callback)
        rospy.loginfo("Started rover6 dronecan bridge")
        while not rospy.is_shutdown():
            try:
                self.dronecan_node.spin(timeout=0.2)
            except:
                pass
        self.dronecan_node.close()
        rospy.loginfo("Rover6 dronecan gateway closed")

    def send_command_callback(self, arg):
        if arg is not None:
            rospy.loginfo("Dronecan command sent")
        else:
            rospy.loginfo("Dronecan command timeout")

    def send_setlid_command(self, node_id, state):
        message = dronecan.com.zxdynamics.controller.SetLid.Request()
        message.state = state
        self.dronecan_node.request(message, node_id, self.send_command_callback)

    def send_setknock_command(self, node_id, state):
        message = dronecan.com.zxdynamics.controller.SetKnock.Request()
        message.state = state
        self.dronecan_node.request(message, node_id, self.send_command_callback)
    
    def send_setled_command(self, node_id, channel, effect, color1, color2):
        message = dronecan.com.zxdynamics.controller.SetLED.Request()
        message.channel = channel
        message.effect = effect
        message.color1=dronecan.com.zxdynamics.controller.RGB888(red=color1[0], green=color1[1], blue=color1[2])
        message.color2=dronecan.com.zxdynamics.controller.RGB888(red=color2[0], green=color2[1], blue=color2[2])
        self.dronecan_node.request(message, node_id, self.send_command_callback)

    def set_lid_service(self, req: SetLidRequest):
        self.send_setlid_command(self.controller_node_id, req.state)
        resp = SetLidResponse()
        resp.success = True
        return resp

    def set_knock_service(self, req: SetBoolRequest):
        self.send_setknock_command(self.controller_node_id, req.data)
        resp = SetBoolResponse()
        resp.success = True
        return resp
    
    def set_led_service(self, req: SetLEDRequest):
        color1 = [req.color1.r * 0xFF, req.color1.g * 0xFF, req.color1.b * 0xFF]
        color2 = [req.color2.r * 0xFF, req.color2.g * 0xFF, req.color2.b * 0xFF]
        self.send_setled_command(self.controller_node_id, req.channel, req.effect, color1, color2)
        resp = SetLEDResponse()
        resp.success = True
        return resp

    def rover6_controller_status_callback(self, event):
        controller_status_message = ControllerStatus()
        controller_status_message.lid_state = event.message.lid_state
        controller_status_message.controller_error = event.message.controller_error
        controller_status_message.lid_angle = event.message.lid_angle
        controller_status_message.base_sensor_angle = event.message.base_sensor_angle
        controller_status_message.lid_sensor_angle = event.message.lid_sensor_angle
        controller_status_message.motor1_current = event.message.motor1_current
        controller_status_message.motor2_current = event.message.motor2_current
        controller_status_message.motor1_voltage = event.message.motor1_voltage
        controller_status_message.motor2_voltage = event.message.motor2_voltage
        self._ros_pub_controller_status.publish(controller_status_message)

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
    rospy.init_node('rover6_dronecan')
    rospy.loginfo("Starting rover6 dronecan bridge")
    rospy.loginfo("Loading custom DSDL")
    dronecan.load_dsdl(rospkg.RosPack().get_path('rover6_dronecan') + "/dsdl/com/")
    bridge = Rover6Dronecan()
    bridge.run()
