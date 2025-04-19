from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import copy

class MsgProducer:

    def __init__(self):
        
        self.top_height = 0.6589699317712477
        self.disk_thickness = 0.020
        self.base_cartesians = {}
        self.set_up_base_cartesians()
        self.grab_cartesians = {}
        self.set_up_grab_cartesians()

    def set_up_base_cartesians(self):

        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = -0.3463386680520093
        pose_msg.pose.position.y = 0.7085564847436839
        pose_msg.pose.position.z = 0.550762193872306

        # Set the orientation
        pose_msg.pose.orientation.x = -0.005135652915416821 
        pose_msg.pose.orientation.y = 0.700599465406546
        pose_msg.pose.orientation.z = 0.7134869226760107
        pose_msg.pose.orientation.w = -0.008391978998458685

        self.base_cartesians["a"] = pose_msg

        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = -0.16023952117840168
        pose_msg.pose.position.y = 0.7081600369982457
        pose_msg.pose.position.z = 0.5475750870920807

        # Set the orientation
        pose_msg.pose.orientation.x = -0.005135652915416821 
        pose_msg.pose.orientation.y = 0.700599465406546
        pose_msg.pose.orientation.z = 0.7134869226760107
        pose_msg.pose.orientation.w = -0.008391978998458685

        self.base_cartesians["b"] = pose_msg


        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = 0.027097420286928464
        pose_msg.pose.position.y = 0.7110830903885906
        pose_msg.pose.position.z = 0.54756436203263

        # Set the orientation
        pose_msg.pose.orientation.x = -0.005135652915416821 
        pose_msg.pose.orientation.y = 0.700599465406546
        pose_msg.pose.orientation.z = 0.7134869226760107
        pose_msg.pose.orientation.w = -0.008391978998458685

        self.base_cartesians["c"] = pose_msg

    def set_up_grab_cartesians(self):

        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = -0.3463598787075253
        pose_msg.pose.position.y = 0.9056499938766277
        pose_msg.pose.position.z = 0.5507606182192675

        # Set the orientation
        pose_msg.pose.orientation.x = -0.005135652915416821 
        pose_msg.pose.orientation.y = 0.700599465406546
        pose_msg.pose.orientation.z = 0.7134869226760107
        pose_msg.pose.orientation.w = -0.008391978998458685

        self.grab_cartesians["a"] = pose_msg

        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = -0.16025886523952607
        pose_msg.pose.position.y = 0.9138899451971881
        pose_msg.pose.position.z = 0.5475881372822955

        # Set the orientation
        pose_msg.pose.orientation.x = -0.005135652915416821 
        pose_msg.pose.orientation.y = 0.700599465406546
        pose_msg.pose.orientation.z = 0.7134869226760107
        pose_msg.pose.orientation.w = -0.008391978998458685

        self.grab_cartesians["b"] = pose_msg


        # Create the PoseStamped message
        pose_msg = PoseStamped()

        # Set the header
        pose_msg.header = Header()
        pose_msg.header.frame_id = 'base_link'

        # Set the position
        pose_msg.pose.position.x = 0.027113516884134697
        pose_msg.pose.position.y = 0.9149684524328227
        pose_msg.pose.position.z = 0.5475803913659942

        # Set the orientation
        pose_msg.pose.orientation.x = -0.005135652915416821 
        pose_msg.pose.orientation.y = 0.700599465406546
        pose_msg.pose.orientation.z = 0.7134869226760107
        pose_msg.pose.orientation.w = -0.008391978998458685

        self.grab_cartesians["c"] = pose_msg

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
            copied_msg.pose.position.x = self.grab_cartesians[pole].pose.position.x
            copied_msg.pose.position.y = self.grab_cartesians[pole].pose.position.y

        return copied_msg
