#!/usr/bin/env python3

from queue import PriorityQueue
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from playsound import playsound

from dis_task1.msg import MoverMessage
from sweeper import Sweeper
from robot_commander import RobotCommander

class MoveCommand:

    priorities = {
        "sweep": 2000,
        "ring": 1000,
        "face": 0,
    }

    def __init__(self, location : PoseStamped, type : str, data : str):
        self.location = location
        self.priority = MoveCommand.priorities.get(type, 0)
        if (type in MoveCommand.priorities):
            MoveCommand.priorities[type] += 1
        
        self.data = data

    def completed(self, node):
        if (self.data is None or len(self.data) == 0):
            node.publish_marker(self.coordinates(), self,  0.1, 1, [0.5, 0.5, 0.5])
            return
        
        node.get_logger().info(f"Reached goal with data: {self.data}")
        node.publish_marker(self.coordinates(), self,  0.3, 3, [0.5, 0.5, 0.5])
        playsound(f"src/dis_task1/audio/{self.data}")

    def coordinates(self):
        return [self.location.pose.position.x, self.location.pose.position.y, self.location.pose.position.z]

    def __lt__(self, other):
        return self.priority < other.priority

class Mover(Node):

    detected_colors = []

    def __init__(self):
        super().__init__("mover")
        self.command_queue = PriorityQueue()
        self.command_subscriber = self.create_subscription(MoverMessage, "/mover_command", self.parse_move_message, 10)
        self.marker_publisher = self.create_publisher(Marker, "/command_marker", 10)
        self.update_timer = self.create_timer(2, self.timer_callback)
        self.robot_commander = RobotCommander()
        self.current_command : MoveCommand = None

        self.get_logger().info("Mover constructed!")
        self.get_logger().info("Zzzing for 10 seconds")
        time.sleep(10)

        sweeper = Sweeper()
        messages : list[MoverMessage] = sweeper.pipeline()
        sweeper.destroy_node()
        for i, message in enumerate(messages):
            if (i % 2 == 0):
                continue
            self.parse_move_message(message)
            time.sleep(0.1)
        
    def new_command(self):
        self.current_command : MoveCommand = self.command_queue.get()
        self.robot_commander.goToPose(self.current_command.location)

    def parse_move_message(self, message : MoverMessage):
        self.get_logger().info(f"Received command of type: {message.type}, data: {message.data}")
        
        if (message.type == "sweep"):
            move_command = MoveCommand(message.location, "sweep", None)
            self.publish_marker(move_command.coordinates(), move_command, 0.1, 1, [0., 1., 0.])
        elif (message.type == "ring"):
            if (message.data in Mover.detected_colors):
                return
            Mover.detected_colors.append(message.data)
            playsound(f"src/dis_task1/audio/{message.data}.mp3")
            return
            #move_command = MoveCommand(message.location, "ring", f"{message.data}.mp3")
            #self.publish_marker(move_command.coordinates(), move_command, 0.4, [1., 0., 0.])
        elif (message.type == "face"):
            move_command = MoveCommand(message.location, "face", "hello_human.mp3")
            self.publish_marker(move_command.coordinates(), move_command, 0.3, 3, [0., 0., 1.])
        else:
            raise ValueError("Invalid move type")

        self.command_queue.put(move_command)

        if self.current_command is None:
            self.new_command()
            return

        # Check if queue's head priority is lesser (importanter) than current active command
        current_head : MoveCommand = self.command_queue.queue[0]
        if self.current_command.priority < current_head.priority:
            return
        
        # If it is, requeue current command (the one with importanter priority) and set queue's head as current command
        self.robot_commander.cancelTask()
        self.robot_commander.destroyNode()
        self.robot_commander = RobotCommander()
        self.command_queue.put(self.current_command)
        
        self.new_command()

    def publish_marker(self, location : list[float], move_command : MoveCommand, size : int, shape : int, color : list[float] = [0.5, 0.5, 0.5]):
        marker = Marker()

        marker.header.frame_id = "map"

        marker.type = shape
        marker.id = move_command.priority

        marker.scale.x = float(size)
        marker.scale.y = float(size)
        marker.scale.z = float(size)

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        marker.pose.position.x = float(location[0])
        marker.pose.position.y = float(location[1])
        marker.pose.position.z = float(location[2])

        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        self.marker_publisher.publish(marker)

    def timer_callback(self):
        if self.current_command is None or not self.robot_commander.isTaskComplete():
            self.get_logger().info("Navigating...")
            return

        self.get_logger().info("Goal position reached")
        self.current_command.completed(self)

        self.new_command()
        
def main():
    rclpy.init(args=None)
    node = Mover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
            


        
    