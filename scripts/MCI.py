#!/usr/bin/env python
import sys
import copy
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from ros_myo.msg import MyoPose


mc = []

# ROS stuff
class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        hand_group = moveit_commander.MoveGroupCommander("hand")
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = arm_group.get_planning_frame()

        # We can also print the name of the end-effector link for this group:
        eef_link = arm_group.get_end_effector_link()

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.hand_group = hand_group
        self.arm_group = arm_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def place(self, x, y, z):
        place_list =[]
        place_location = moveit_msgs.msg.PlaceLocation()
        place_location.place_pose.header.frame_id = "panda_link0"
        place_location.place_pose.pose.position.x = x
        place_location.place_pose.pose.position.y = y
        place_location.place_pose.pose.position.z = z
        place_list.append(copy.deepcopy(place_location))

        place_location.pre_place_approach.direction.header.frame_id = "panda_link0"
        place_location.pre_place_approach.direction.vector.z = -1.0
        place_location.pre_place_approach.min_distance = 0.095
        place_location.pre_place_approach.desired_distance = 0.115
        place_list.append(copy.deepcopy(place_location))

        place_location.post_place_retreat.direction.header.frame_id = "panda_link0"
        place_location.post_place_retreat.direction.vector.y = -1.0
        place_location.post_place_retreat.min_distance = 0.1
        place_location.post_place_retreat.desired_distance = 0.25
        place_list.append(copy.deepcopy(place_location))

        self.arm_group.place("box", place_location)
        self.open_gripper()
        self.detach_box()

    def pick(self, x, y, z):
        grasp_list = []
        eef_pose = moveit_msgs.msg.Grasp()
        eef_pose.grasp_pose.header.frame_id = "hand"
        eef_pose.grasp_pose.pose.orientation.y = 0
        grasp_list.append(copy.deepcopy(eef_pose))

        grasp = moveit_msgs.msg.Grasp()
        grasp.grasp_pose.header.frame_id = "panda_link0"
        grasp.grasp_pose.pose.orientation.x = 0
        grasp.grasp_pose.pose.orientation.y = 1
        grasp.grasp_pose.pose.orientation.z = 0

        grasp.grasp_pose.pose.position.x = x
        grasp.grasp_pose.pose.position.y = y
        grasp.grasp_pose.pose.position.z = z
        grasp_list.append(copy.deepcopy(grasp))

        grasp.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        grasp.pre_grasp_approach.direction.vector.x = 1
        grasp.pre_grasp_approach.min_distance = 0.095
        grasp.pre_grasp_approach.desired_distance = 0.116
        grasp_list.append(copy.deepcopy(grasp))

        grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        grasp.post_grasp_retreat.direction.vector.z = 1
        grasp.post_grasp_retreat.min_distance = 0.1
        grasp.post_grasp_retreat.desired_distance = 0.25
        grasp_list.append(copy.deepcopy(grasp))

        self.arm_group.pick("box", grasp_list)
        self.close_gripper()
        self.attach_box()

    def close_gripper(self):
        self.hand_group.set_named_target("close")
        plan2 = self.hand_group.go()
        self.hand_group.stop()
        self.attach_box()

    def open_gripper(self):
        self.hand_group.set_named_target("open")
        plan2 = self.hand_group.go()
        self.hand_group.stop()
        self.detach_box()

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # functions makes sure the object appears by waiting for a status update
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4):
        # target_size = [0.07, 0.07, 0.07]
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.orientation.z = 2.5
        box_pose.pose.position.x = 0.5
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = 0.035
        self.box_name = "box"
        self.scene.add_box(self.box_name, box_pose, size=(0.05, 0.05, 0.07))
        rospy.sleep(1)

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        grasping_group = 'hand'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)

        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def plan_cartesian_path(self, scale=1):
        waypoints = []

        wpose = self.arm_group.get_current_pose().pose
        wpose.position.x += scale * 0.2  # First move forward (x)
        wpose.position.z -= scale * 0.27  # and then down (z)
        waypoints.append(copy.deepcopy(wpose))

        # wpose.position.z -= scale * 0.25  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))

        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):
        self.arm_group.execute(plan, wait=True)

    def go_to_pose_goal(self, x, y, z):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0
        pose_goal.orientation.x = 1
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z


        # hand_pose = self.arm_group.get_current_pose(self.eef_link).pose
        # hand_pose.orientation.y = -0.38
        # (plan, fraction) = self.arm_group.compute_cartesian_path(hand_pose, 0.01, 0)

        self.arm_group.set_pose_target(pose_goal)
        plan1 = self.arm_group.go(wait=True)  # plan
        self.arm_group.stop()  # stop movement
        self.arm_group.clear_pose_targets()  # clear pose targets

        # current_pose = self.arm_group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)

def all_close(goal, actual, tolerance):
    """
      Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
      @param: goal       A list of floats, a Pose or a PoseStamped
      @param: actual     A list of floats, a Pose or a PoseStamped
      @param: tolerance  A float
      @returns: bool
      """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

# MYO stuff (MC = morse code)

def MC_logic(gest):
    global mc
    if gest.pose == 2:  # FIST / DOT
        mc.append(0)
        # rospy.loginfo("Wave-in")
        print(mc)
    elif gest.pose == 5:  # FINGERS_SPREAD / DASH
        mc.append(1)
        # rospy.loginfo("Wave-out")
        print(mc)
    elif gest.pose == 1:  # FIST / DONE
        MC_commands(mc)
        mc = []

def MC_commands(mc):
# OPEN/CLOSE
    if mc == [0]:
        # open gripper
        rospy.loginfo("OPEN Gripper")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.open_gripper()
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [1]:
        # close gripper
        rospy.loginfo("CLOSE Gripper")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.close_gripper()
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")

# 4 positions
    elif mc == [0, 0]:
        # GO to the right
        rospy.loginfo("GO Right")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.go_to_pose_goal(0, 0.5, 0.25)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [0, 1]:
        # GO to the left
        rospy.loginfo("GO Left")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.go_to_pose_goal(0, -0.5, 0.25)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [1, 0]:
        # GO to the front
        rospy.loginfo("GO Front")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.go_to_pose_goal(0.5, 0, 0.25)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [1, 1]:
        # GO to the back
        rospy.loginfo("GO Back")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.go_to_pose_goal(-0.5, 0, 0.25)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")

# 4 positions for PICK/PLACE
    elif mc == [0, 0, 0]:
        # PLACE to the right
        rospy.loginfo("PLACE down box")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.place(0, 0.5, 0.1)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
        pass
    elif mc == [0, 0, 1]:
        # PLACE to the left
        rospy.loginfo("PLACE down box")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.place(0, -0.5, 0.1)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [0, 1, 0]:
        # PLACE to the front
        rospy.loginfo("PLACE down box")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.place(0.5, 0, 0.1)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [0, 1, 1]:
        # PLACE to the back
        rospy.loginfo("PLACE down box")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.place(-0.5, 0, 0.1)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [1, 0, 0]:
        # GO to the right
        rospy.loginfo("PLACE down box")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.place(0, 0.5, 0.1)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [1, 0, 1]:
        # GO to the left
        rospy.loginfo("PICK up box")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.pick(0, -0.5, 0.15)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [1, 1, 0]:
        # GO to the front
        rospy.loginfo("PICK up box")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.pick(0.5, 0, 0.15)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")
    elif mc == [1, 1, 1]:
        # GO to the back
        rospy.loginfo("PICK up box")
        tutorial = MoveGroupPythonIntefaceTutorial()
        tutorial.pick(-0.5, 0, 0.15)
        rospy.sleep(0.5)
        rospy.loginfo("Ready for next command")



if __name__ == '__main__':
    tutorial = MoveGroupPythonIntefaceTutorial()
    rospy.sleep(1)
    tutorial.add_box()

    pose_sub = rospy.Subscriber('/myo_raw/myo_gest', MyoPose, MC_logic, queue_size=1)
    rospy.loginfo("Awaiting publications...")
    rospy.spin()

    # def MC_logic(gest):
    #     global mc
    #     if gest.pose == 3:  # WAVE_IN
    #         mc.append(0)
    #         rospy.loginfo("Wave-in")
    #         print(mc)
    #         MC_commands(mc=mc)
    #
    #     elif gest.pose == 4:  # WAVE_OUT
    #         mc.append(1)
    #         rospy.loginfo("Wave-out")
    #         print(mc)
    #         MC_commands(mc=mc)



        # if len(mc) == 2 and mc == [0, 0]:
        #     print("Moving to the front")
        #     tutorial = MoveGroupPythonIntefaceTutorial()
        #     tutorial.go_to_pose_goal(0.5, 0, 0.25)
        #     mc = []
        #     print("Ready for next command")
        #
        # # back
        # if len(mc) == 2 and mc == [0, 1]:
        #     print("Moving to the back")
        #     tutorial = MoveGroupPythonIntefaceTutorial()
        #     tutorial.go_to_pose_goal(-0.5, 0, 0.25)
        #     mc = []
        #     print("Ready for next command")
        #
        # # right
        # if len(mc) == 2 and mc == [1, 0]:
        #     print("Moving to the right")
        #     tutorial = MoveGroupPythonIntefaceTutorial()
        #     tutorial.go_to_pose_goal(0, 0.5, 0.25)
        #     mc = []
        #     print("Ready for next command")
        #
        # # left
        # if len(mc) == 2 and mc == [1, 1]:
        #     print("Moving to the left")
        #     tutorial = MoveGroupPythonIntefaceTutorial()
        #     tutorial.go_to_pose_goal(0, -0.5, 0.25)
        #     mc = []
        #     print("Ready for next command")





    # tutorial.go_to_pose_goal(0.5, 0, 0.25)
    # myo startup stuff
    # myo = Myo(sys.argv[1] if len(sys.argv) >= 2 else None)
    # myo.connect()
    # myo.vibrate(2)
    # myo.add_pose_handler(lambda p: myo.MC_logic(p))

    # while True:
    #     if myo.run(1) is None:
    #         print("Connection lost, trying to reconnect")
    #         myo.connect()
    #         time.sleep(1)
    #         myo.vibrate(1)
    #         print(myo.run(1))




        # moveit commands
        # go to specified position

        # cartesian_path, fraction = tutorial.plan_cartesian_path()
        # tutorial.execute_plan(cartesian_path)
        # rospy.sleep(1)

        # tutorial.go_to_pose_goal(0.5, 0, 0.25)

        # tutorial.pick(0.5, 0, 0.15)
        # tutorial.close_gripper()
        # rospy.sleep(1)
        #
        # tutorial.place(0, 0.5, 0.1)
        # tutorial.open_gripper()


