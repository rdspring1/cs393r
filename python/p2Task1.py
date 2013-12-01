from state import * 
import commands, core, util, pose, percepts, kicks
import time
import math

stand = True

class Task1(StateMachine):
  def setup(self):
    start = Node()
    choose = ChooseNode()
    wait = WaitNode()
    finish = Node()
    
    self._adt(start, N, choose)
    self._adt(choose, S(Choices.Wait), wait, S, choose)
    self._adt(choose, S(Choices.Finish), finish)

class Choices:
  Start = 0
  Wait = 1
  Choose = 2
  Finish = 3
  NumChoices = 4

class ChooseNode(Node):
  def run(self):  
    global stand 
    if(stand):
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


