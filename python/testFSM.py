from state import * 
import commands, core, head, util, pose
import time
import math

CTHRESHOLD = 20
BTHRESHOLD = math.radians(20)
ATHRESHOLD = 350
DTHRESHOLD = 40
kp = 0.3

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    choose = ChooseNode()
    self._adt(start, N, choose)
    self._adt(choose, S(Choices.Stand), StandNode(), S, choose)
    self._adt(choose, C, finish)

class Choices:
  Localize = 0
  Rotate = 1
  Forward = 2
  Reverse = 3
  Stand = 4
  NumChoices = 5

class ChooseNode(Node):
  def __init__(self):
     super(ChooseNode, self).__init__()

  def run(self):
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    print "** Current Location X: %d Y: %d" % (robot.loc.x, robot.loc.y)
    print "** Confidence: " + str(robot.radius)
    print "** Distance: " + str(robot.visionDistance)
    print "** Bearing: " + str(robot.visionBearing)
    print "** Orientation: " + str(robot.orientation)
    print "** RADIUS: " + str(robot.radius)
    commands.stand()
    
    if self.getTime() > 4:
        if (robot.radius < 0.40 and robot.radius > 0.0):
            self.postSignal(Choices.Stand)
        elif (robot.radius < 0.65 and robot.radius > 0.0):
            commands.stand()
        elif robot.visionDistance > DTHRESHOLD:
            print "_______Distance __"
            value = min(0.001 * robot.visionDistance, 0.30)
            commands.setWalkVelocity(value, 0.0, kp*robot.visionBearing)
        else:
           commands.stand()
           print "_____Localization Complete______ " + str(robot.visionDistance)
           core.speech.say("Localization Complete")
           self.postSignal(Choices.Stand)

        if (self.getTime() > 10 and robot.visionDistance < ATHRESHOLD and robot.visionDistance > DTHRESHOLD):
            core.speech.say("Approach")
            self.postSignal(Choices.Stand)
            
class StandNode(Node):
  def run(self):
    print "______ SCAN _____"
    commands.stand()
    if self.getTime() > 1.0:
      self.postSuccess()
