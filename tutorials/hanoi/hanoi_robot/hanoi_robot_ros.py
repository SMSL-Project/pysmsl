#!/usr/bin/env python3

import sys
sys.path.append('src/smsl/')
from functools import partial
import json
import smsl
from smsl_state import smslState
sys.path.append('tutorials/hanoi/hanoi_robot/')
from msg_producer import MsgProducer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped


class smslTutorialHanoiSensor:

    def __init__(self, sm, disks_weights) -> None:
        """
        disks_weights: {"1": w1, "2": w2, "3": w3}
            The weights of the physical disks
        """
        self.sm = sm
        self.disks_weights = disks_weights
        self.state_mapper = {}
        self.initialize_state_mapper()
        self.pole_name = {'a': 0, 'b': 1, 'c': 2}

    def initialize_state_mapper(self):
        """
        Iterate through all possible states and get a mapper
        from weights to states
        """
        for node in self.sm.state_machine[0].graph.nodes:
            cur_readings = {"a": 0.0, "b": 0.0, "c": 0.0}
            # eg: aaa, abc, ccc (pole name)
            state_digits = node.state_name[-3:]

            for i in range(len(state_digits)):  # i here is the disk type-1
                cur_readings[state_digits[i]] += self.disks_weights[str(i+1)]

            # json dump to make it hashable
            cur_readings_str = json.dumps(cur_readings)
            if cur_readings_str in self.state_mapper:
                raise Exception(
                    "Repeated weights configuration. Try a different set. The states to weight sets mapping have to be 1 to 1!")
            else:
                self.state_mapper[cur_readings_str] = state_digits
                print("Added " + cur_readings_str + " : " + state_digits)

    def get_states(self, readings):
        """
        readings: {"a": r1, "b": r2, "c": r3}
        """

        closest_readings = None
        min_similarity = float('inf')
        for possible_reading_str in self.state_mapper.keys():
            possible_reading = json.loads(possible_reading_str)
            similarity = (readings['a']-possible_reading['a'])**2 \
                        + (readings['b']-possible_reading['b'])**2 \
                        + (readings['c']-possible_reading['c'])**2
            if min_similarity > similarity:
                min_similarity = similarity
                closest_readings = possible_reading

        return self.state_mapper[json.dumps(closest_readings)]
    
    def get_readings(self):
        """
        Interface to the sensor readings
        """
        # data = get_topic_data_once('load_cells_data', Float32MultiArray)
        # print(f"Received data: {data}")
        # return {"a": data[0], "b": data[1], "c": data[2]}
        return {"a": 114000.0, "b": 106000.0, "c": 0.0}


class smslTutorialHanoiRobot:
    """
    Tutorial of a Hanoi game robot
    """
    def __init__(self) -> None:
        self.buffer_move_disk = []
        self.msg_producer = MsgProducer()
        self.action = []
        self.action_str = []

    def move(self, start, end):
        """
        Move robot in Cartesian
        """
        pass

    def move_disk(self, disk, target, cur_state):
        """
        The sequence of an action moving disk to target
        """

        print("Move " + str(disk) + " to " + str(target) + ":")

        # Move to top of the disk
        s = "Move to top of " + str(disk) + " (pole " + cur_state[int(disk)-1] + ")"
        print(s)
        self.action_str.append(s)
        self.action.append(
            self.msg_producer.get_cartesians(cur_state, cur_state[int(disk)-1], True)
        )

        # Move to disk
        disk_thickness = 0.038671
        height = {"a": 0.0, "b": 0.0, "c": 0.0}
        for i in cur_state:
            height[i] = height[i] + disk_thickness
        s = "Get down to height " + str(height[cur_state[int(disk)-1]]-disk_thickness) + " " + cur_state
        print(s)
        self.action_str.append(s)
        self.action.append(
            self.msg_producer.get_cartesians(cur_state, cur_state[int(disk)-1], False)
        )

        # Grab disk
        s = "Close jaw"
        print(s)
        self.action_str.append(s)
        self.action.append(
            self.msg_producer.get_cartesians(cur_state, cur_state[int(disk)-1], False, True)
        )

        # Move to top of the disk
        s = "Get up to top height"
        print(s)
        self.action_str.append(s)
        self.action.append(
            self.msg_producer.get_cartesians(cur_state, cur_state[int(disk)-1], True, False, True)
        )

        # Move to top of the target
        s = "Move to top of pole " + str(target)
        print(s)
        self.action_str.append(s)
        self.action.append(
            self.msg_producer.get_cartesians(cur_state, target, True, False, True)
        )

        # Move to the target
        s = "Get down to height " + str(height[target])
        print(s)
        self.action_str.append(s)
        self.action.append(
            self.msg_producer.get_cartesians(cur_state, target, False, False, True)
        )

        # Release disk
        s = "Open jaws"
        print(s)
        self.action_str.append(s)
        self.action.append(
            self.msg_producer.get_cartesians(cur_state, target, False, False, False, True)
        )

        print()

        # Check if there is still remaining
        if len(self.buffer_move_disk) > 0:
            fnc = self.buffer_move_disk.pop(0)
            fnc()
        else:
            for i in range(len(self.action_str)):
                print(self.action_str[i])
                print(self.action[i])

    def go(self, ops, sts):
        for i in range(len(ops)):
            self.Op_nx(ops[i][0], ops[i][1], sts[i])
        fnc = self.buffer_move_disk.pop(0)
        fnc()

    def Op_nx(self, disk, target, cur_state):
        self.buffer_move_disk.append(partial(self.move_disk, disk, target, cur_state))


class TargetPosesPublisher(Node):
    def __init__(self, robot):
        super().__init__('target_poses_publisher')
        # Create a publisher on the "target_poses" topic.
        self.publisher_ = self.create_publisher(PoseArray, 'target_poses', 10)
        self.timer = self.create_timer(1.0, self.publish_target_poses)
        self.robot = robot

    def publish_target_poses(self):
        # Create a PoseArray message
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        for target in self.robot.action:
            if isinstance(target, PoseStamped):
                msg.poses.append(target.pose)
            else:
                # If your array is not PoseStamped, you may have a different message type.
                self.get_logger().warn('Unexpected message type in robot.action.target_pose_list')

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info('Published target poses.')

        # destroy the timer afterward.
        self.timer.cancel()

def main(args=None):
    
    # Read in SMSL
    sm = smsl.smslStateMachine('examples/hanoi.json')
    
    # Disks' weights
    disks_weights = {"1": 43000, "2": 75000, "3": 98000}

    # Initialize the robot and sensor
    robot = smslTutorialHanoiRobot()
    sensor = smslTutorialHanoiSensor(sm, disks_weights)

    # Get states and calculate path
    start = sensor.get_states(sensor.get_readings())
    end = 'ccc'
    path = sm.state_machine[0].shortest_edge_path(
        smslState('State_'+start),
        smslState('State_'+end)
    )
    path_states = sm.state_machine[0].shortest_path(
        smslState('State_'+start),
        smslState('State_'+end)
    )

    # Buffer the operation sequence
    ops = []
    for op in path:
        _op = op[0]['OP']
        ops.append((_op.split('_')[1][0], _op.split('_')[1][1]))

    print()
    print(start + " to " + end)
    print(ops)

    sts = []
    for st in path_states:
        _st = st.state_name
        sts.append(_st.split('_')[1])
    print(sts)
    print()
    
    # Start the robot commands
    robot.go(ops, sts)

    # Publish
    rclpy.init(args=args)
    node = TargetPosesPublisher(robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
