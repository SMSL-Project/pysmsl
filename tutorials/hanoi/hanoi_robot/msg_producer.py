from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import copy

class MsgProducer:

    def __init__(self):
        self.base_cartesians = {}
        self.set_up_base_cartesians()
        self.top_height = 0.3902996289675307
        self.grab_y = 0.48687487626632897
        self.disk_thickness = 0.038671
        self.set_up_base_cartesians()

    def set_up_base_cartesians(self):

        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = -0.21188165741609216
        pose_msg.pose.position.y = 0.41949083332240383
        pose_msg.pose.position.z = 0.24920643514825566

        # Set the orientation
        pose_msg.pose.orientation.x = 0.1425637908061899
        pose_msg.pose.orientation.y = 0.6909234219109138
        pose_msg.pose.orientation.z = 0.6904252762890299
        pose_msg.pose.orientation.w = -0.16004164603970708

        self.base_cartesians["a"] = pose_msg

        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = -0.03438751179550842
        pose_msg.pose.position.y = 0.41949083332240383
        pose_msg.pose.position.z = 0.24920643514825566

        # Set the orientation
        pose_msg.pose.orientation.x = 0.1425637908061899
        pose_msg.pose.orientation.y = 0.6909234219109138
        pose_msg.pose.orientation.z = 0.6904252762890299
        pose_msg.pose.orientation.w = -0.16004164603970708

        self.base_cartesians["b"] = pose_msg


        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = 0.13733056350988745
        pose_msg.pose.position.y = 0.41949083332240383
        pose_msg.pose.position.z = 0.24920643514825566

        # Set the orientation
        pose_msg.pose.orientation.x = 0.1425637908061899
        pose_msg.pose.orientation.y = 0.6909234219109138
        pose_msg.pose.orientation.z = 0.6904252762890299
        pose_msg.pose.orientation.w = -0.16004164603970708

        self.base_cartesians["c"] = pose_msg


    def get_cartesians(self, cur_state, pole, top_height=False, grabbing=False, grabbed=False, releasing=False):
        copied_msg = copy.deepcopy(self.base_cartesians[pole])

        if top_height:
            copied_msg.pose.position.z = self.top_height

        else: 
            if grabbing:
                height = {"a": 0.0, "b": 0.0, "c": 0.0}
                for i in cur_state:
                    height[i] += self.disk_thickness
                copied_msg.pose.position.z = \
                    copied_msg.pose.position.z + height[pole]-self.disk_thickness
            
            elif grabbed:
                height = {"a": 0.0, "b": 0.0, "c": 0.0}
                for i in cur_state:
                    height[i] += self.disk_thickness
                copied_msg.pose.position.z = \
                    copied_msg.pose.position.z + height[pole]
                
            elif releasing:
                height = {"a": 0.0, "b": 0.0, "c": 0.0}
                for i in cur_state:
                    height[i] += self.disk_thickness
                copied_msg.pose.position.z = \
                    copied_msg.pose.position.z + height[pole]
                
            elif (not grabbing) and (not grabbed) and (not releasing):
                height = {"a": 0.0, "b": 0.0, "c": 0.0}
                for i in cur_state:
                    height[i] += self.disk_thickness
                copied_msg.pose.position.z = \
                    copied_msg.pose.position.z + height[pole]-self.disk_thickness

        if grabbed or grabbing:
            copied_msg.pose.position.y = self.grab_y

        return copied_msg
