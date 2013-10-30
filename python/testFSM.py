from state import * 
import commands, core, head, util, pose
import time
import math

CTHRESHOLD = 20
BTHRESHOLD = math.radians(10)
DTHRESHOLD = 50
kp = 0.2

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    choose = ChooseNode()
    self._adt(start, N, choose)
    self._adt(choose, S(Choices.Localize), LocalizeNode(), S, choose)
    self._adt(choose, C, finish)

class Choices:
  Localize = 0
  Rotate = 1
  Forward = 2
  Reverse = 3
  NumChoices = 4

class ChooseNode(Node):
  def __init__(self):
     super(ChooseNode, self).__init__()
     self.task = head.Scan(maxPan = 2.0, period = 7.0, numSweeps = 1, direction = 1)

  def run(self):
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    print "Current Location X: %d Y: %d" % (robot.loc.x, robot.loc.y, )
    print "Confidence: " + str(robot.visionConfidence)
    print "Distance: " + str(robot.visionDistance)
    print "Bearing: " + str(robot.visionBearing)
    commands.stand()
    if robot.visionConfidence > CTHRESHOLD:
        print "Localization"
        self.task.processFrame()
    elif robot.visionBearing > BTHRESHOLD and (2 * math.pi - robot.visionBearing) > BTHRESHOLD:
        print "Vision Bearing"
        commands.setWalkVelocity(0.0, 0.0, kp * robot.visionBearing)
    elif robot.visionDistance > DTHRESHOLD:
        print "Distance"
        commands.setWalkVelocity(kp * robot.visionDistance, 0.0, 0.0)
    else:
       commands.stand()
       print "Localization Complete"

class LocalizeNode(Node):
  def __init__(self):
     super(LocalizeNode, self).__init__()
     self.task = head.Scan(maxPan = 2.0, period = 7.0, numSweeps = 1, direction = 1)

  def run(self):
     self.task.processFrame()
     if self.task.finished():
        print "Scan Head"
        self.postSuccess()

  def reset(self):
     super(LocalizeNode, self).reset()
     self.task = head.Scan(maxPan = 2.0, period = 7.0, numSweeps = 1, direction = 1)
     
