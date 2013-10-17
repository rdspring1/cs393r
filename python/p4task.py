from state import * 
import commands, core, util, pose, percepts, kicks
import time
import math
import head

class Task1(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    choose = ChooseNode()
    
    self._adt(start, N, choose)
    self._adt(choose, S(Choices.Search), SearchNode(), S, PauseNode(), S, choose)
    self._adt(choose, S(Choices.Finish), finish)

class Choices:
  Choose = 0
  Search = 1
  Pause = 2
  Finish = 3
  NumChoices = 4

class ChooseNode(Node):
  def run(self):  
    core.speech.say("CHOOSE")
    pink_yellow = core.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW)
    pink_blue = core.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE)
    yellow_pink = core.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)
    yellow_blue = core.world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE)
    blue_yellow = core.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW)
    blue_pink = core.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)
    self.postSignal(Choices.Search)

class SearchNode(Node):
  def run(self):
    core.speech.say("SEARCH")
    print "SearchNode"
    commands.stand()
    commands.setWalkVelocity(0.25, 0.0, 0.0)
    commands.setStiffness()
    if self.getTime() > 2.0:
       commands.setHeadPan(-45 * core.DEG_T_RAD, 1.5, True)

    if self.getTime() > 7.0:
       commands.setHeadPan(45 * core.DEG_T_RAD, 1.5, True)

    if self.getTime() > 11.0:
       self.postSuccess()

class PauseNode(Node):
  def run(self):
    core.speech.say("PAUSE")
    commands.stand()
    if self.getTime() > 5.0:
       self.postSuccess()
