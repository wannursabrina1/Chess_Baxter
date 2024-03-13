#!/usr/bin/env python
import sys,tf
import copy

import rospy
import rospkg

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander
from copy import deepcopy

class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

        # Get the current pose of the robot's limb
        current_pose = self._limb.endpoint_pose()

        # Calculate the Euclidean distance between the current pose and the start angles
        distance = ((current_pose['position'].x - start_angles.position.x)**2 + 
                    (current_pose['position'].y - start_angles.position.y)**2 + 
                    (current_pose['position'].z - start_angles.position.z)**2)**0.5

        # Check if the robot has reached the starting position 
        if distance < 0.01: 
            # Return True if the robot has reached the start position
            return True
        else:
            # Return False if the robot has not yet reached the start position
            return False

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)   

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    # Calling the spawn service
    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    # Initial Pose to spawn the chess pieces
    initial_chess_pose = Pose(position=Point(x=0.6, y=0.6, z=0.8))
    list_pieces = rospy.get_param('list_pieces')
    board_setup = rospy.get_param('board_setup')
    pieces_xml = rospy.get_param('pieces_xml')
    piece_names = rospy.get_param('piece_names')
    piece_positionmap = rospy.get_param('piece_target_position_map')

    limb = 'left'
    hover_distance = 0.15  # meters

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
    # we need to compensate for this offset which is 0.93 from the ground in gazebo to
    # the actual 0, 0, 0 in Rviz.
    starting_pose = Pose(position=Point(x=0.7, y=0.135, z=0.35), orientation=overhead_orientation)

    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    # Move to Starting Position
    pnp.move_to_start(starting_pose)

    # If the baxter robot is at the start position, then spawn chess piece
    if pnp.move_to_start(starting_pose):
        next_piece = None
        for row, each in enumerate(board_setup):
            for col, piece in enumerate(each): 

                # spawn the next chess piece in the list
                if next_piece is not None and next_piece in pieces_xml:

                    print srv_call("%s%d" % (next_piece, col), pieces_xml[next_piece], "", initial_chess_pose, "world")
                    next_piece = None

                if piece in list_pieces:
                    piece_names.append("%s%d" % (piece,col))

                    # pick up the chess piece from initial position
                    pick_up_pose = (Pose(position=Point(x=0.6, y=0.6, z=-0.14),orientation=overhead_orientation))

                    key = str(row) + str(col)
                    place_chess = piece_positionmap[key]
                    # place the chess piece on the chessboard setup position
                    place_pose = (Pose(position=Point(x=place_chess[0], y=place_chess[1], z = -0.14),orientation=overhead_orientation))

                    pnp.pick(pick_up_pose)
                    pnp.place(place_pose)

                # Check if there is a next piece in the current row
                if col + 1 < len(each):  
                    next_piece = each[col + 1]

        # grid references to pick up white rook, black pawn, white knight, black bishop, white pawn and black queen
        pick_list = ['77', '14', '71','05', '64','01']
        # grid references for 6 hardcoded chess move
        place_list = ['57', '24', '52','14','54','20']

        # pick and place for baxter robot to play chess
        for pick, place in zip(pick_list, place_list):
            
            pick_piece = piece_positionmap[pick]
            pick_pose = Pose(position=Point(x=pick_piece[0], y=pick_piece[1], z=pick_piece[2]), orientation=overhead_orientation)

            place_piece = piece_positionmap[place]
            place_pose = Pose(position=Point(x=place_piece[0], y=place_piece[1], z=place_piece[2]), orientation=overhead_orientation)

            if rospy.is_shutdown():
                break

            print("\nPicking...")
            pnp.pick(pick_pose)

            print("\nPlacing...")
            pnp.place(place_pose)

    # If the baxter robot is not at the start position, then wait until it reaches
    else:
        print("Waiting for robot to reach starting position...")
        while not pnp.move_to_start(starting_pose):
            rospy.sleep(1.0)  # wait for 1 second
        print("Robot has reached starting position.")


    return 0
    

if __name__ == '__main__':
    sys.exit(main())
