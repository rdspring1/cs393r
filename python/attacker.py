from state import * 
import commands, core, util, pose, percepts, kicks
import time
import math

scoreDistance = 150
angle = 10
kp = 0.3
kd = 0.1
ki = 0.0

previous_error = 0
integralAngleError = 0

previous_ball_angle = 0

previous_ball_distance = 0
ball_distance = 0
didKick = False

class DribbleAndKick(StateMachine):
  def setup(self):
    start = Node()
    choose = ChooseNode()
    search = SearchNode()
    forward = WalkForwardNode()
    detectGoal = DetectGoalNode()
    detectScore = DetectScore()
    finish = Node()
    
    self._adt(start, N, choose)
    self._adt(choose, S(Choices.Search), search, S, choose)
    self._adt(choose, S(Choices.DetectGoal), detectGoal)
    self._adt(choose, S(Choices.WalkForward), forward)
    self._adt(detectGoal, S(Choices.WalkForward), forward)
    self._adt(detectGoal, S(Choices.Search), search)
    self._adt(forward, S(Choices.WalkForward), forward)
    self._adt(forward, S(Choices.Search), search)
    self._adt(forward, S(Choices.DetectGoal), detectGoal)
    self._adt(forward, S(Choices.LeftKick), LeftKickNode(), S, choose)
    self._adt(forward, S(Choices.RightKick), RightKickNode(), S, choose)
    self._adt(detectScore, S(Choices.Choose), choose)
    self._adt(detectScore, S(Choices.Finish), finish)

class Choices:
  WalkForward = 1
  Search = 2
  DetectGoal = 3
  LeftKick = 4
  RightKick = 5
  DetectScore = 6
  Choose = 7
  Finish = 8
  NumChoices = 9

class ChooseNode(Node):
  def run(self):
    global previous_ball_distance
    #print "started"
    if previous_ball_distance == 0:
        commands.stand()
        self.postSignal(Choices.WalkForward)

class WalkForwardNode(Node):
  def run(self):
    global previous_ball_distance
    if self.getTime() > 5:
        print "*** Going to kick"
        #self.postSignal(Choices.RightKick)
        core.kick_request.set(core.Kick.STRAIGHT, core.Kick.RIGHT, 0, 500)
        previous_ball_distance = 1
        self.postSuccess()

        
class DetectGoalNode(Node):
  def run(self):
    global previous_ball_angle
    global previous_ball_distance
    core.speech.say("Detect")
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    print "*** GOAL seen ** " + str(goal.seen)
    print "*** GOAL radius ** " + str(goal.radius)
    
    #PD values for the angle
    align_kp = 0.3
    align_kd = 0.4
    #PD values for the y
    y_kp = 0.1
    y_kd = 0.1
    
    if not(ball.seen):
      core.speech.say("Lost Ball - Reacquire")
      self.postSignal(Choices.Search)
    elif not(goal.seen) or goal.radius < 0.05 or (math.fabs(goal.imageCenterX-ball.imageCenterX) > 40): #search for it
      ball_error_angle = ball.visionBearing
      d_ball_angle = ball_error_angle - previous_ball_angle
      previous_ball_angle = ball_error_angle
      move_angle = align_kp * ball_error_angle + align_kd * d_ball_angle
      ball_error_distance = (ball.visionDistance - ball_distance)/100 #translation to use this in the x parameter for walkVelocity
      d_ball_distance = (ball.visionDistance - previous_ball_distance)/100
      previous_ball_distance = ball.visionDistance
      x = (160 - ball.imageCenterX)/100
      commands.setWalkVelocity(ball_error_distance*y_kp + d_ball_distance*y_kd, 0.3*x, move_angle) #rotate around ball #y=0.3 initially
      print "*** BALL ERROR DISTANCE *** " + str(ball_error_distance)
    else:
      # move to Kick task
      self.postSignal(Choices.WalkForward)

class LeftKickNode(Node):
  def __init__(self):
    super(LeftKickNode, self).__init__()
    self.task = kicks.Kick(core.Kick.LEFT, 5000)
    
  def run(self):
    print "*** Left Kick"
    #core.speech.say("Left Kick")
    commands.stand()
    self.task.processFrame()
    if self.task.finished():
      self.postSuccess()
      
  def reset(self):
    super(LeftKickNode, self).reset()
    self.task = kicks.Kick(core.Kick.LEFT, 5000)

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
      
class DetectScore(Node):
  def run(self):
    if self.getTime() > 5:
       ball = core.world_objects.getObjPtr(core.WO_BALL)
       goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
       dist = math.sqrt((math.pow((goal.imageCenterX - ball.imageCenterX), 2) + math.pow((goal.imageCenterY - ball.imageCenterY), 2)))
       print " ** goalX ** " + str(goal.imageCenterX)
       print " ** goalY ** " + str(goal.imageCenterY)
       print " ** ballX ** " + str(ball.imageCenterX)
       print " ** ballY ** " + str(ball.imageCenterY)
       print " ** Score Distance ** " + str(dist)
       if(ball.fromTopCamera == goal.fromTopCamera and dist < scoreDistance):
          core.speech.say("Scored Goal")
          print "Scored Goal"
          self.postSignal(Choices.Finish)
       else:
          core.speech.say("Missed Goal")
          print "Missed Goal"
          self.postSignal(Choices.Choose)
