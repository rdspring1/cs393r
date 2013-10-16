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
    self._adt(detectGoal, S(Choices.WalkForward), forward)
    self._adt(detectGoal, S(Choices.Search), search)
    self._adt(forward, S(Choices.WalkForward), forward)
    self._adt(forward, S(Choices.Search), search)
    self._adt(forward, S(Choices.DetectGoal), detectGoal)
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
    penalty_line = core.world_objects.getObjPtr(core.WO_OPP_PENALTY)
    print "** imageX ** " + str(ball.imageCenterX)
    print "** imageY ** " + str(ball.imageCenterY)
    print "** error ** " + str(ball.imageCenterX - 160)
    print "** Head Yaw Value ** %f" % percepts.joint_angles[core.HeadYaw]
    print "** SEEN ** " + str(ball.seen)
    print "** TopCamera ** " + str(ball.fromTopCamera)
    
    robotX = 160
    robotY = 240
    diffX = robotX - ball.imageCenterX
    diffY = robotY - ball.imageCenterY
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
        print "Move if valid region"
        # Move toward the ball with PID if its in a valid region, before the line
        print "line seen " + str(penalty_line.seen) + " line radius " + str(penalty_line.radius) + " ball y " + str(ball.imageCenterY) + " line y " + str(penalty_line.imageCenterY)
        if penalty_line.seen and penalty_line.radius > 5000 and ball.imageCenterY < penalty_line.imageCenterY:
            commands.stand()
        else:
            print " ** Moving with PID **"
            errorAngle = ball.visionBearing
            derivativeError = errorAngle - previous_error
            previous_error = errorAngle
            moveAngle = (kp * errorAngle) + (ki * integralAngleError) + (kd * derivativeError) #PID controller
            integralAngleError += errorAngle
            commands.setWalkVelocity(0.30, 0.0, moveAngle)
    else:
      previous_error = 0
      integralAngleError = 0
      self.postSignal(Choices.Search) #search

class WalkForwardNode(Node):
  def run(self):
    global angle
    global previous_error
    global previous_ball_angl
    global ball_distance
    global previous_ball_distance
    global integralAngleError
    core.speech.say("Walk Forward")
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    penalty_line = core.world_objects.getObjPtr(core.WO_OPP_PENALTY)
    robotX = 160
    robotY = 240
    diffX = robotX - ball.imageCenterX
    diffY = robotY - ball.imageCenterY
    commands.setStiffness()
    commands.stand()
    commands.setHeadTilt(-20) #tilt head down

    goalBallError = goal.imageCenterX - ball.imageCenterX
    if not(ball.seen):
      self.postSignal(Choices.Search)
    elif not(goal.seen):
      self.postSignal(Choices.DetectGoal)
    else:
      print "** xaxis **" + str(diffX)
      print "** fromTopCamera **" + str(ball.fromTopCamera)
      print "** yaxis **" + str(ball.imageCenterY)
      print "** goalBall Alignment **" + str(goalBallError)
      print "** goalRadius **" + str(goal.radius)
      if math.fabs(diffX) < 80 and math.fabs(diffX) > 25 and ball.fromTopCamera == False and ball.imageCenterY > 175 and math.fabs(goalBallError) < 70 and goal.radius > 0.11:
        previous_ball_angle = 0
        ball_distance = ball.visionDistance
        previous_ball_distance = ball_distance
        commands.stand()
        if diffX >= 0:
           print " ** Left Kick Ball ** " + str(diffX)
           self.postSignal(Choices.LeftKick)
        else:
           print " ** Right Kick Ball ** " + str(diffX)
           self.postSignal(Choices.RightKick)
      else:
        # Move toward the ball with PID if its in a valid region, before the line
        #print "line seen " + str(penalty_line.seen) + " line radius " + str(penalty_line.radius) + " ball y " + str(ball.imageCenterY) + " line y " + str(penalty_line.imageCenterY)
        if penalty_line.seen and penalty_line.radius > 4000 and ball.imageCenterY < penalty_line.imageCenterY:
            commands.stand()
        else:
            print " ** Move toward the ball with PID **"
            #errorAngle = math.atan((diffX / diffY)) #radians errorAngle
            errorAngle = ball.visionBearing
            derivativeError = errorAngle - previous_error
            previous_error = errorAngle
            integralAngleError += errorAngle
            moveAngle = (kp * errorAngle) + (ki * integralAngleError) + (kd * derivativeError) #PID controller
            integralAngleError += errorAngle
            commands.setWalkVelocity(0.15, 0.01 * goalBallError, moveAngle)
            self.postSignal(Choices.WalkForward)
        
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
    print "Left Kick"
    core.speech.say("Left Kick")
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
    print "Right Kick"
    core.speech.say("Right Kick")
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
