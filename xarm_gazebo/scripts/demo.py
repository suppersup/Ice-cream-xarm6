#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def all_close(goal, actual, tolerance):
  tolerance == 0.1
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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ##information about robot kinematic model and current joint states
    
    robot = moveit_commander.RobotCommander()

    ##initiate a planning scence
    scene = moveit_commander.PlanningSceneInterface()

    ## define the move group and name inside the package
    group_name = "xarm6"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    #display planning frame and name
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    ## print the name for the end effector
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)

    ##list all names of robot
    group_names = robot.get_group_names()
    print ("============ Available Planning Groups:", robot.get_group_names())

   
    ##robot state for debugging
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")
    


    
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  

## planning cartesian path
  def plan_cartesian_path(self, scale=1):
    
    move_group = self.move_group

  
    waypoints = []
    
    wpose = move_group.get_current_pose().pose

    #wpose.position.x = 0.0  #point A
    #wpose.position.y = 0.0
    #wpose.position.z = 1.0
    #waypoints.append(copy.deepcopy(wpose))
    
    #wpose.position.x = 0.432  #point B
    #wpose.position.y = -0.492
    #wpose.position.z = 0.635
    #waypoints.append(copy.deepcopy(wpose))

    #wpose.position.x = 0.055  # point C
    #wpose.position.y = -0.398
    #wpose.position.z = 0.483
    #waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = -0.118  # point E
    wpose.position.y = -0.387
    wpose.position.z = 0.475
    waypoints.append(copy.deepcopy(wpose))
    
    #wpose.position.x = 0.372  # Point F
    #wpose.position.y = 0.343
    #wpose.position.z = 0.462
    #waypoints.append(copy.deepcopy(wpose))

    
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction


##display the path plan
  def display_trajectory(self, plan):
    
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish incase there is an error
    display_trajectory_publisher.publish(display_trajectory)

   

##define executing path
  def execute_plan(self, plan):
  
    move_group = self.move_group

    move_group.execute(plan, wait=True)



 ## define the run the main function when called 
def main():
  try:
    print ("AME 547 :D ")
    
    tutorial = MoveGroupPythonIntefaceTutorial()

    
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    
    tutorial.display_trajectory(cartesian_plan)

    
    tutorial.execute_plan(cartesian_plan)


    print ("Your Ice-cream is ready :D")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
