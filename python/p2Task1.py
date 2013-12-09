from state import * 
import commands, core, util, pose, percepts, kicks
import time
import math

stand = True
num = 0

class Task1(StateMachine):
  def setup(self):
    start = Node()
    choose = ChooseNode()
    wait = WaitNode()
    finish = Node()
    walk = WalkNode()
    
    self._adt(start, N, choose)
    self._adt(choose, S(Choices.Wait), wait, S, choose)
    self._adt(choose, S(Choices.Walk), walk, S, choose)
    self._adt(choose, S(Choices.Finish), finish)

class Choices:
  Start = 0
  Wait = 1
  Choose = 2
  Finish = 3
  Walk = 4
  NumChoices = 5


class ChooseNode(Node):
  def run(self):  
    global stand 

    if stand:
        core.walk_request.exit_balance_ = False
        commands.stand()
        stand = False
        self.postSignal(Choices.Wait)

    #walk and stop, resume after 3 seconds
    if(core.walk_request.exit_balance_):
        self.postSignal(Choices.Walk)
    
    '''if stand:
        commands.stand()
        stand = False
        self.postSignal(Choices.Wait)

    if core.walk_request.exit_step_:
        print "______EXIT STEP" 
        commands.stand()
        self.postSignal(Choices.Wait)'''

class WalkNode(Node):
    def run(self):
        #core.speech.say("walk")
        commands.setWalkVelocity(0.5, 0.0, -20*core.DEG_T_RAD) #rotate around ball #y=0.3 initially
        if self.getTime() > 3:
            commands.stand()
            core.speech.say("stand")
            self.postSuccess()
            

class WaitNode(Node):
  def run(self):
    if self.getTime() > 5:
      print "______READY" 
      core.speech.say("ready")
      core.walk_request.exit_step_ = False
      core.walk_request.exit_balance_ = True
      self.postSuccess()
