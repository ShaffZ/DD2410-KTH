#!/usr/bin/env python

from termios import B75
import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from reactive_sequence import RSequence
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class BehaviourTree(ptr.trees.BehaviourTree):
    
    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")


		# tuck the arm
        b0 = tuckarm()

		# move the head down
        b1 = movehead("down")

        # detect if cube is on table
        b2 = detect()

		# pick the cube
        b3 = pick()

		# turn to the second table
        turn1 = pt.composites.Selector(
			name="Turn to table 1 fallback",
			children=[counter(38, "Turned?"), go("Turn around!", 0, -0.5)]
		)

        # move the cube to the second table
        go1 = pt.composites.Selector(
			name="Go to table 1 fallback",
			children=[counter(6, "At table?"), go("Go to table!", 1, 0)]
		)

        # Movement in one branch
        b4 = pt.composites.Sequence(name="moving 1 sequence", children=[turn1, go1])
        
		# place the cube
        b5 = place()

        # turn to the first table
        turn2 = pt.composites.Selector(
			name="Turn to table 2 fallback",
			children=[counter(25, "Turned?"), go("Turn around!", 0, -0.5)]
		)

        # move the cube to the first table
        go2 = pt.composites.Selector(
			name="Go to table 2 fallback",
			children=[counter(5, "At table 2?"), go("Go to table 2!", 1, 0)]
		)

        # Movement in one branch
        b6 = pt.composites.Sequence(name="moving 2 sequence", children=[turn2, go2])


        # CHECK IF THE CUBE IS PLACED ON SECOND TABLE or go back on failure
        
        b7 = pt.composites.Sequence(name="Check if done fallback", children=[b5, check()])


        b8 = pt.composites.Selector(name="place and check, or move Fallback", children=[b7, b6])



		# become the tree
        tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b8])
        
        super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)


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
            self.detected = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, timeout=30)

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
            rospy.loginfo("done picking")

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
        self.done = False
        self.tried = False

        # become a behaviour
        super(place, self).__init__("Place cube!")

    def update(self):

        if self.done:
            return pt.common.Status.SUCCESS

            # try if not tried

            # command
        self.place_req = self.place_srv()

            # tell the tree you're running
            #return pt.common.Status.RUNNING

        # if succesful
        if self.place_req.success:
            self.done = True
            rospy.loginfo("done placing")

            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_req.success:
            rospy.loginfo("Failed sdadadas placing")
            return pt.common.Status.FAILURE

        # if still trying
        #else:
            #return pt.common.Status.RUNNING


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