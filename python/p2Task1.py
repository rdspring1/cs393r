from state import * 
import commands, core, util, pose, percepts, kicks
import time
import math

scoreDistance = 15
angle = 10
kp = 0.2
kd = 0.1
ki = 0.0

previous_error = 0
integralAngleError = 0

previous_ball_angle = 0

previous_ball_distance = 0
ball_distance = 0

class Task1(StateMachine):
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
    self._adt(detectGoal, S(Choices.WalkForward), forward)
    self._adt(forward, S(Choices.WalkForward), forward)
    self._adt(forward, S(Choices.LeftKick), LeftKickNode(), S, detectScore)
    self._adt(forward, S(Choices.RightKick), RightKickNode(), S, detectScore)
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
    global angle
    global previous_error
    global previous_ball_angle
    global ball_distance
    global previous_ball_distance
    global integralAngleError
    
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    '''print "** SEEN ** " + str(ball.seen)
    print "** imageX ** " + str(ball.imageCenterX)
    print "** imageY ** " + str(ball.imageCenterY)
    print "** error ** " + str(ball.imageCenterX - 160)
    print "** Head Yaw Value ** %f" % percepts.joint_angles[core.HeadYaw]
    print "** TopCamera ** " + str(ball.fromTopCamera)'''
    
    commands.stand()
    
    '''robotX = 160
    robotY = 240
    diffX = robotX - ball.imageCenterX
    diffY = robotY - ball.imageCenterY
    commands.stand()
    self.postSignal(Choices.RightKick)
    if ball.seen:
      core.speech.say("Seen")
      print "** DIFFX ** " + str(math.fabs(diffX))
      print "** DIFFY ** " + str(math.fabs(diffY))

      if math.fabs(diffX) < 45 and ball.fromTopCamera == False and ball.imageCenterY > 90:
        # reset variables before going to Goal Detection
        integralAngleError = 0
        previous_ball_angle = 0
        ball_distance = ball.visionDistance
        previous_ball_distance = ball_distance
        self.postSignal(Choices.DetectGoal)
      else:
        # Move toward the ball with PID
        errorAngle = math.atan((diffX / diffY)) #radians errorAngle
        derivativeError = errorAngle - previous_error
        previous_error = errorAngle
        moveAngle = (kp * errorAngle) + (ki * integralAngleError) + (kd * derivativeError) #PID controller
        integralAngleError += errorAngle
        print "** ERROR ANGLE ** %f" % errorAngle
        commands.setWalkVelocity(0.0, 0.0, moveAngle)
    else:
      previous_error = 0
      integralAngleError = 0
      self.postSignal(Choices.Search) #search
      '''

class WalkForwardNode(Node):
  def run(self):
    global angle
    global previous_error
    global previous_ball_angle
    global ball_distance
    global previous_ball_distance
    global integralAngleError
    core.speech.say("Walk Forward")
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    robotX = 160
    robotY = 240
    diffX = robotX - ball.imageCenterX
    diffY = robotY - ball.imageCenterY
    commands.setStiffness()
    commands.stand()
    commands.setHeadTilt(-20) #tilt head down

    goalBallError = goal.imageCenterX - ball.imageCenterX
    if ball.seen:
      print "** xaxis **" + str(math.fabs(diffX))
      print "** fromTopCamera **" + str(ball.fromTopCamera)
      print "** yaxis **" + str(ball.imageCenterY)
      print "** goalBall Alignment **" + str(goalBallError)
      print "** goalRadius **" + str(goal.radius)
      if math.fabs(diffX) < 45 and ball.fromTopCamera == False and ball.imageCenterY > 175 and math.fabs(goalBallError) < 65 and goal.radius > 0.35:
        print "Kick Ball"
        previous_ball_angle = 0
        ball_distance = ball.visionDistance
        previous_ball_distance = ball_distance
        commands.stand()
        if diffX >= 0:
           self.postSignal(Choices.RightKick)
        else:
           self.postSignal(Choices.LeftKick)
      
      else:
        # Move toward the ball with PID
        errorAngle = math.atan((diffX / diffY)) #radians errorAngle
        derivativeError = errorAngle - previous_error
        previous_error = errorAngle
        moveAngle = (kp * errorAngle) + (ki * integralAngleError) + (kd * derivativeError) #PID controller
        integralAngleError += errorAngle
        commands.setWalkVelocity(0.0, 0.01 * goalBallError, moveAngle)
        print "** WALK FORWARD ERROR ANGLE ** %f" % errorAngle
        self.postSignal(Choices.WalkForward)
        
class DetectGoalNode(Node):
  def run(self):
    global previous_ball_angle
    global previous_ball_distance
    core.speech.say("Detect")
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    print "*** GoAL seen ** " + str(goal.seen)
    print "*** GoAL radius ** " + str(goal.radius)
    
    #PD values for the angle
    align_kp = 0.3
    align_kd = 0.4
    #PD values for the y
    y_kp = 0.1
    y_kd = 0.1
    
    if not(goal.seen) or goal.radius < 0.06 or (math.fabs(goal.imageCenterX-ball.imageCenterX) > 40): #search for it
      ball_error_angle = ball.visionBearing
      d_ball_angle = ball_error_angle - previous_ball_angle
      previous_ball_angle = ball_error_angle
      move_angle = align_kp * ball_error_angle + align_kd * d_ball_angle

      ball_error_distance = (ball.visionDistance - ball_distance)/100 #translation to use this in the x parameter for walkVelocity
      d_ball_distance = (ball.visionDistance - previous_ball_distance)/100
      previous_ball_distance = ball.visionDistance
      x = (160 - ball.imageCenterX)/100
      print "*** BALL ERROR DISTANCE *** " + str(ball_error_distance)
      commands.setWalkVelocity(ball_error_distance*y_kp + d_ball_distance*y_kd, 0.3*x, move_angle) #rotate around ball #y=0.3 initially
    else:
      core.speech.say("Goal")
      commands.stand()
      # move to Kick task
      self.postSignal(Choices.WalkForward)
      
class LeftKickNode(Node):
  def __init__(self):
    super(LeftKickNode, self).__init__()
    self.task = kicks.Kick(core.Kick.LEFT, 700)
    
  def run(self):
    print "Left Kick"
    core.speech.say("Left Kick")
    commands.stand()
    self.task.processFrame()
    if self.task.finished():
      self.postSuccess()
      
class RightKickNode(Node):
  def __init__(self):
    super(RightKickNode, self).__init__()
    self.task = kicks.Kick(core.Kick.RIGHT, 700)
    
  def run(self):
    print "Right Kick"
    core.speech.say("Right Kick")
    commands.stand()
    self.task.processFrame()
    if self.task.finished():
      self.postSuccess()
    
class SearchNode(Node):
  def run(self):
    core.speech.say("Search")
    commands.setWalkVelocity(0.0, 0.0, 20 * core.DEG_T_RAD)
    if self.getTime() > 2:
      self.postSuccess()
      
class DetectScore(Node):
  def run(self):
    core.speech.say("Goal Scored!")
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    diff = goal.visionDistance - ball.visionDistance
    print " ** Score Distance ** " + str(diff)
    self.postSignal(Choices.Finish)
