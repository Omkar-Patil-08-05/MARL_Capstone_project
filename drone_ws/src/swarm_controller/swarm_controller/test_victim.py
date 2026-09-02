import rclpy
from rclpy.node import Node
import time
from victim_manager import VictimManager

rclpy.init()
node = Node("test_node")
vm = VictimManager(node)

vm._add_victim("victim_1", 20.0, 20.0, profile=False)

print("First Detection")
vm.process_visual_detection(20.0, 20.0, "MOCK", 0.99, "0")
print(vm.get_dashboard_state())

time.sleep(1)

print("Second Detection")
vm.process_visual_detection(20.5, 20.5, "MOCK", 0.95, "1")
print(vm.get_dashboard_state())

rclpy.shutdown()
