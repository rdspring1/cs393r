from state import * 
import commands, core, head, util, pose
import time
import math

CTHRESHOLD = 0.80
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
  _choices = 0
  def run(self):
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    print "Current Location X: %d Y: %d" % (robot.loc.x, robot.loc.y, )
    print "Confidence: " + str(robot.visionConfidence)
    if robot.visionConfidence < CTHRESHOLD:
        self.postSignal(Choices.Localize)
    elif robot.visionBearing > BTHRESHOLD and (2 * math.pi - robot.visionBearing) > BTHRESHOLD:
        commands.setWalkVelocity(0.0, 0.0, kp * robot.visionBearing)
    elif robot.visionDistance > DTHRESHOLD:
	commands.setWalkVelocity(kp * robot.visionDistance, 0.0, 0.0)
    else:
        commands.stand()
	print "Localization Complete"
        core.speech.say("Localization Complete")

class LocalizeNode(Node):
  def __init__(self):
     super(LocalizeNode, self).__init__()
     self.task = head.Scan()

  def run(self):
     print "Scan Head"
     core.speech.say("Scan Head")
     self.task.processFrame()
     if self.task.finished():
        self.postSuccess()

  def reset(self):
     super(LocalizeNode, self).reset()
     self.task = head.Scan()
     
