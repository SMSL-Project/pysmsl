import sys
sys.path.append('../../../src/smsl/')

from functools import partial
import json
import smsl
from smsl_state import smslState

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
        return {"a": 114000.0, "b": 106000.0, "c": 0.0}


class smslTutorialHanoiRobot:
    """
    Tutorial of a Hanoi game robot
    """
    def __init__(self) -> None:
        self.buffer_move_disk = []

    def move(self, start, end):
        """
        Move robot in Cartesian
        """
        pass

    def move_disk(self, disk, target):
        """
        The sequence of an action moving disk to target
        """
        print("Move " + str(disk) + " to " + str(target))

        # Move to top of the disk

        # Move to disk

        # Grab disk

        # Move to top of the disk

        # Move to top of the target

        # Move to the target

        # Release disk

        # Check if there is still remaining
        if len(self.buffer_move_disk) > 0:
            fnc = self.buffer_move_disk.pop(0)
            fnc()

    def go(self, ops):
        for i in ops:
            self.Op_nx(i[0], i[1])
        fnc = self.buffer_move_disk.pop(0)
        fnc()

    def Op_nx(self, disk, target):
        self.buffer_move_disk.append(partial(self.move_disk, disk, target))


def main():

    # Read in SMSL
    sm = smsl.smslStateMachine('../../../examples/hanoi.json')
    
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

    # Buffer the operation sequence
    ops = []
    for op in path:
        _op = op[0]['OP']
        ops.append((_op.split('_')[1][0], _op.split('_')[1][1]))

    print(start + " to " + end)
    print(ops)
    
    # Start the robot commands
    robot.go(ops)

if __name__ == "__main__":
    main()
