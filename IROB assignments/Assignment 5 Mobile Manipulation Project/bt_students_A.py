#!/usr/bin/env python

from termios import B110
import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from reactive_sequence import RSequence
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

head_pose = "down"


class BehaviourTree(ptr.trees.BehaviourTree):
    def __init__(self):
        rospy.loginfo("Initialising behaviour tree")

        b0 = tuckarm()
        b1 = localize()
        # b2 = pt.composites.Selector(name= "navigate to the cube", children=[navigate_to_cube(), relocalize()])
        b2 = navigate_to_cube()
        b3 = movehead("down")
        b4 = detect()
        b5 = pick()
        b6 = movehead("up")
        # b7 = pt.composites.Selector(name= "navigate to the table", children=[navigate_to_table(), relocalize()])
        b7 = navigate_to_table()
        b8 = place()
        b9 = check()

        # b9 = pt.composites.Sequence(name="Check if done fallback", children=[b8, check()])
        b10 = reset_cube()
        # final = pt.composites.Sequence(name="Repeat", children=[b2,b3, b4, b5, b6, b7, b8])
        # b11 = pt.composites.Sequence(name="navigate back to cube", children=[b10, b6, final])
        # b12 = pt.composites.Selector(name="place and check", children=[b9, b11])

        # tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6, b1, b7, b12])
        loop = pt.composites.Sequence(name="Loop", children=[b3, b4, b5, b6, b7, b8, b9])
        select = pt.composites.Selector(name="selector", children=[loop, b10])
        tree = RSequence(name="Main sequence", children=[b0, b1, b2, select])
        super(BehaviourTree, self).__init__(tree)

        rospy.sleep(5)
        self.setup(timeout = 10000)
        while not rospy.is_shutdown():
            self.tick_tock(1)


class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):
        # success if done

        print("Tried", self.tried)
        print("Done", self.done)
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            rospy.loginfo("Head is done")

            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class reset_cube(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.cube_ModelState = ModelState()
        self.cube_ModelState.model_name = "aruco_cube"   # b6 = movehead("up")
        self.cube_ModelState.pose.position.x = -1.130530
        self.cube_ModelState.pose.position.y = -6.653650
        self.cube_ModelState.pose.position.z = 0.86250
        self.cube_ModelState.pose.orientation.x = 0
        self.cube_ModelState.pose.orientation.y = 0
        self.cube_ModelState.pose.orientation.z = 0
        self.cube_ModelState.pose.orientation.w = 1
        self.cube_ModelState.twist.linear.x = 0
        self.cube_ModelState.twist.linear.y = 0
        self.cube_ModelState.twist.linear.z = 0
        self.cube_ModelState.twist.angular.x = 0
        self.cube_ModelState.twist.angular.y = 0
        self.cube_ModelState.twist.angular.z = 0
        self.cube_ModelState.reference_frame = "map"
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        self.direction = "up"
        super(reset_cube, self).__init__("reset cube")

    def update(self):
        rospy.loginfo("reset the cube")
        self.move_head_req = self.move_head_srv(self.direction)
        self.reset_cube_request = self.set_model_state(self.cube_ModelState)
        return pt.common.Status.FAILURE


class localize(pt.behaviour.Behaviour):
    def __init__(self):
        local_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.local_srv = rospy.ServiceProxy(local_srv_nm, Empty)
        self.amcl_top = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_top, Twist, queue_size=10)
        self.clear_costmaps_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.clear_costmaps_srv = rospy.ServiceProxy(self.clear_costmaps_srv_nm, Empty)
        self.move_msg = Twist()
        self.move_msg.angular.z = -1
        self.move_msg.linear.x = 0
        self.rate = rospy.Rate(10)
        self.localized = False
        self.reset = False
        self.count = 0
        self.local_srv()
        rospy.loginfo("Initializing localization service")
        super(localize, self).__init__("Localization")

    def check_convergence(self, cloud):
        if max(cloud.pose.covariance) < 0.05:
            return True
        else:
            return False

    def update(self):
        if self.localized == True and self.reset == False:
            return pt.common.Status.SUCCESS

        if self.reset == True:
            self.reset == False
            self.localized == False
            self.move_msg = Twist()
            self.move_msg.angular.z = -1
            self.move_msg.linear.x = 0
            self.local_srv()

        self.cmd_vel_pub.publish(self.move_msg)
        self.rate.sleep()

        while not rospy.is_shutdown() and self.count < 60:
            self.count += 1
            return pt.common.Status.RUNNING

        self.move_msg = Twist()
        self.cmd_vel_pub.publish(self.move_msg)

        cloud = rospy.wait_for_message(self.amcl_top, PoseWithCovarianceStamped, 5)
        print(cloud)
        if self.check_convergence(cloud):
            self.localized = True
            rospy.loginfo("points cloud converged")
            self.clear_costmaps_srv()
            return pt.common.Status.SUCCESS
        else:
            self.localized = False        
            self.move_msg = Twist()
            self.move_msg.angular.z = -1
            self.counter = 0
            rospy.loginfo("points cloud diverge")
            return pt.common.Status.FAILURE

class navigate_to_cube(pt.behaviour.Behaviour):
    def __init__(self):
        self.navigated = False
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.pick_pose = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)
        self.amcl_top = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        self.goal = MoveBaseGoal()

        local_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.local_srv = rospy.ServiceProxy(local_srv_nm, Empty)
        cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_top, Twist, queue_size=10)
        self.clear_costmaps_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.clear_costmaps_srv = rospy.ServiceProxy(self.clear_costmaps_srv_nm, Empty)
        self.move_msg = Twist()
        self.move_msg.angular.z = -1
        self.move_msg.linear.x = 0
        self.rate = rospy.Rate(10)
        self.localized = False
        self.reset = False
        self.count = 0
        rospy.loginfo("Initializing navigation to cube service")
        super(navigate_to_cube, self).__init__("Navigation to cube")

    def check_convergence(self, cloud):
        if max(cloud.pose.covariance) < 0.05:
            return True
        else:
            return False

    def update(self):
        if self.navigated == True:
            return pt.common.Status.SUCCESS

        self.goal.target_pose = self.pick_pose
        self.move_base_ac.send_goal(self.goal)
        success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(60))

        if success_navigation:
            self.navigated = True
            return pt.common.Status.SUCCESS
        elif success_navigation == False:
            self.move_base_ac.cancal_goal()

            cloud = rospy.wait_for_message(self.amcl_top, PoseWithCovarianceStamped, 5)
            print(cloud)

            if not self.check_convergence(cloud):
                rospy.log("kidnapped")

                self.move_base_ac.cancel_all_goals()
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

class navigate_to_table(pt.behaviour.Behaviour):
    def __init__(self):
        self.navigated = False
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        self.pick_pose = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)
        self.goal = MoveBaseGoal()
        rospy.loginfo("Initializing navigation to table service")
        super(navigate_to_table, self).__init__("Navigation to table")

    def update(self):
        rospy.loginfo("hello")
        if self.navigated == True:
            return pt.common.Status.SUCCESS

        self.goal.target_pose = self.pick_pose
        self.move_base_ac.send_goal(self.goal)
        success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(60))

        if success_navigation:
            self.navigated = True
            rospy.loginfo("nav succeeded")

            return pt.common.Status.SUCCESS
        elif success_navigation == False:
            self.move_base_ac.cancal_goal()
            rospy.loginfo("nav failed")

            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.RUNNING

class detect(pt.behaviour.Behaviour):

    """
    Detects the cube location.
    """
    def __init__(self):

        rospy.loginfo("Initialising detect behaviour.")

        # server
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.detected = None

        # become a behaviour
        super(detect, self).__init__("Detect cube!")

    def update(self):


        try:
            self.detected = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 5)

        except:
            self.detected = None

        # success if done
        if self.detected:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.detected:
            # tell the tree you're running
            return pt.common.Status.FAILURE



class pick(pt.behaviour.Behaviour):

    """
    Picks the cube off of the table.
    """

    def __init__(self):

        rospy.loginfo("Initialising pick behaviour.")

        # Initiating the service call
        pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_srv = rospy.ServiceProxy( pick_srv_nm, SetBool)
        rospy.wait_for_service(pick_srv_nm, timeout=30)

        # service response state
        self.picking = False
        self.done = False

        # become a behaviour
        super(pick, self).__init__("Pick cube!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

            # command
        self.pick_req = self.pick_srv()

        # if succesful
        if self.pick_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_req.success:
            return pt.common.Status.FAILURE



class place(pt.behaviour.Behaviour):

    """
    Places the cube on table.
    """

    def __init__(self):

        rospy.loginfo("Initialising place behaviour.")

        # Initiating the service call
        place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_srv = rospy.ServiceProxy( place_srv_nm, SetBool)
        rospy.wait_for_service(place_srv_nm, timeout=30)

        # service response state
        self.placing = False
        self.done = False

        # become a behaviour
        super(place, self).__init__("Place cube!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

            # command
        self.place_req = self.place_srv()

        # if succesful
        if self.place_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_req.success:
            return pt.common.Status.FAILURE

class check(pt.behaviour.Behaviour):

    """
    Checks if the cube is dropped at right location.
    """
    def __init__(self):

        rospy.loginfo("Initialising check behaviour.")

        # server
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.checked = None

        # become a behaviour
        super(check, self).__init__("Check cube!")

    def update(self):


        try:
            self.checked = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, timeout=30)

        except:
            self.checked = None

        # success if done
        if self.checked:
            rospy.loginfo("CHECK DONEEEE")
            return pt.common.Status.SUCCESS 

        # try if not tried
        elif not self.checked:

            # tell the tree you're running
            return pt.common.Status.FAILURE


	
if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()