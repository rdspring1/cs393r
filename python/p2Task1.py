from state import * 
import commands, core, util, pose, percepts, kicks
import time
import math
import cfgkick

stand = True
num = 0

class Task1(StateMachine):
  def setup(self):
    start = Node()
    choose = ChooseNode()
    wait = WaitNode()
    finish = Node()
    right_kick = RightKickNode()
    
    self._adt(start, N, choose)
    self._adt(choose, S(Choices.Wait), wait, S, choose)
    self._adt(choose, S(Choices.RightKick), right_kick, S, choose)
    self._adt(choose, S(Choices.Finish), finish)

class Choices:
  Start = 0
  Wait = 1
  Choose = 2
  Finish = 3
  RightKick = 4
  NumChoices = 5

class ChooseNode(Node):
  def run(self):  
    global stand 
    global backamount

    if stand:
        commands.stand()
        stand = False
        self.postSignal(Choices.Wait)

class WaitNode(Node):
  def run(self):
    if self.getTime() > 5:
      print "______READY" 
      core.speech.say("ready")
      core.walk_request.start_balance_ = True
      self.postSuccess()

class RightKickNode(Node):
  def __init__(self):
    super(RightKickNode, self).__init__()
    self.task = kicks.Kick(core.Kick.RIGHT, 5000)
    
  def run(self):
    #print "Right Kick"
    #core.speech.say("Right Kick")
    commands.stand()
    self.task.processFrame()
    if self.task.finished():
      self.postSuccess()
      
  def reset(self):
    super(RightKickNode, self).reset()
    self.task = kicks.Kick(core.Kick.RIGHT, 5000)
    
class SearchNode(Node):
  def run(self):
    core.speech.say("Search")
    commands.setWalkVelocity(0.0, 0.0, 20 * core.DEG_T_RAD)
    if self.getTime() > 2:
      self.postSuccess()








