from state import * 
import commands, core, util,  pose, percepts
import time
import math

angle = 10
kp = 0.001
kd = 0.1
ki = 0.0

previous_error = 0
integralAngleError = 0

previous_ball_angle = 0

previous_ball_distance = 0
ball_distance = 0

prev_robot_angle = 0

ballmove = False
prevX = 0
prevY = 0
movethreshold = 40
maxthreshold = 100
block_distance_threshold = 1000
block_velocity_threshold = 35
regular_block_threshold = 275
critical_distance_threshold = 400
center_kp = 0.0002
    
class Choices:
  Move = 0
  Squat = 1
  BlockLeft = 3
  BlockRight = 4
  Search = 5
  NumChoices = 6

# Task approach ball within Penalty Box
class ApproachBall(StateMachine):
    def setup(self):
        start = Node()
        choose = MoveNode()

        self._adt(start, N, choose)
        self._adt(choose, S(Choices.Move), choose)
        self._adt(choose, S(Choices.Search), SearchNode(), S, choose)

# Task block
class Goalie(StateMachine):
  def setup(self):
    start = Node()
    choose = ChooseNode()

    self._adt(start, N, StandNode(), C, choose)
    self._adt(choose, S(Choices.Squat), SquatNode(), S, choose)
    self._adt(choose, S(Choices.BlockLeft), BlockLeftNode(), C, choose)
    self._adt(choose, S(Choices.BlockRight), BlockRightNode(), C, choose)

class ChooseNode(Node):
  def run(self):
    core.speech.say("CHOOSE")
    global ballmove
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    penalty_line = core.world_objects.getObjPtr(core.WO_OPP_PENALTY)    

    commands.stand()
    print "** currentX: " + str(ball.lowerHeight) + " ** futureX: " + str(ball.height);
    print "** currentY: " + str(ball.upperHeight) + " ** futureY: " + str(ball.width);

    #print "______SEEN BALL ____ " + str(ball.seen)
    if not ballmove and ball.seen and ball.lowerHeight > 0:
       distance = math.sqrt(math.pow(ball.lowerHeight - prevX, 2) + math.pow(ball.upperHeight - prevY, 2))
       print "** ballmove: " + str(distance)
       if distance > movethreshold:
          ballmove = True

    if ballmove and ball.seen and ball.lowerHeight > 0:
        angle = ball.radius
        ball_velocity = ball.lowerHeight - ball.height
        distance_to_ball = ball.visionDistance #math.sqrt(math.pow(ball.lowerHeight - 0, 2) + math.pow(ball.upperHeight - 0, 2))
        print "** ballvelocity: " + str(ball_velocity)
        print "** balldistance: " + str(distance_to_ball)
        if ball_velocity >= block_velocity_threshold and ball.height <= critical_distance_threshold:
           core.speech.say("BLOCK")
           # Determine which side the ball is going to pass by the robot
           if ball.width <= 0:
               #print "RIGHT BLOCK"
               #core.speech.say("RIGHT BLOCK")
               self.postSignal(Choices.BlockRight)
           elif ball.width > 0:
               #print "LEFT SIDE"
               #core.speech.say("LEFT BLOCK")
               self.postSignal(Choices.BlockLeft)
        else:
           
           offcenter_error = diffX = 160 - ball.imageCenterX
           print "offcenter: " + str(offcenter_error)

           if not penalty_line.seen or penalty_line.radius < 2000 or penalty_line.imageCenterY < 160:
              core.speech.say("RETREAT")
              commands.setWalkVelocity(-0.30, 0.0, kd * ball.visionBearing)
           elif math.fabs(offcenter_error) < 20:
              commands.stand()
           else:
              core.speech.say("CENTER")
              commands.setWalkVelocity(0, center_kp * offcenter_error, kd * ball.visionBearing)
    else:
       commands.stand()

class SquatNode(Node):
  def __init__(self):
    super(SquatNode, self).__init__()
    self.task = pose.Squat()

  def run(self):
    self.task.processFrame() 
    if self.task.finished() and self.getTime() > 10.0:
       core.speech.say("SQUAT FINISH")
       self.postSuccess()

  def reset(self):
    super(SquatNode, self).reset()
    self.task = pose.Squat()

class BlockLeftNode(Node):
  def __init__(self):
    super(BlockLeftNode, self).__init__()
    self.task = pose.BlockLeft()

  def run(self):
    self.task.processFrame() 
    if self.task.finished() or self.getTime() > 5.0:
       core.speech.say("LEFT BLOCK FINISH")
       self.postCompleted()

  def reset(self):
    super(BlockLeftNode, self).reset()
    self.task = pose.BlockLeft()

class BlockRightNode(Node):
  def __init__(self):
    super(BlockRightNode, self).__init__()
    self.task = pose.BlockRight()

  def run(self):
    self.task.processFrame() 
    if self.task.finished() or self.getTime() > 5.0:
        core.speech.say("RIGHT BLOCK FINISH")
        self.postCompleted()

  def reset(self):
    super(BlockRightNode, self).reset()
    self.task = pose.BlockRight()

class MoveNode(Node):
    def run(self):
       global angle
       global previous_error
       global previous_ball_angl
       global ball_distance
       global previous_ball_distance
       global integralAngleError
       core.speech.say("Walk Forward")
       ball = core.world_objects.getObjPtr(core.WO_BALL)
       penalty_line = core.world_objects.getObjPtr(core.WO_OPP_PENALTY)
       diffX = 160 - ball.imageCenterX
       diffY = 240 - ball.imageCenterY
       commands.setStiffness()
       commands.stand()
       commands.setHeadTilt(-20) #tilt head down

       if not(ball.seen):
          self.postSignal(Choices.Search)
       elif penalty_line.seen and penalty_line.radius > 4000 and ball.imageCenterY < penalty_line.imageCenterY:
          commands.stand()
       else:
          #print " ** Move toward the ball with PID **"
          errorAngle = ball.visionBearing
          derivativeError = errorAngle - previous_error
          previous_error = errorAngle
          integralAngleError += errorAngle
          moveAngle = (kp * errorAngle) + (ki * integralAngleError) + (kd * derivativeError) #PID controller
          integralAngleError += errorAngle
          commands.setWalkVelocity(0.10, 0.01 * goalBallError, moveAngle)
          self.postSignal(Choices.Move)

class SearchNode(Node):
  def run(self):
    core.speech.say("Search")
    commands.setWalkVelocity(0.0, 0.0, 20 * core.DEG_T_RAD)
    if self.getTime() > 2:
      self.postSuccess()

class StandNode(Node):
  def __init__(self):
    super(StandNode, self).__init__()
    self.task = pose.Stand()
 
  def run(self):
    global ballmove
    global prevX
    global prevY
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    ballmove = False
    self.task.processFrame()
    if self.getTime() > 5.0 and self.task.finished():
      prevX = ball.lowerHeight
      prevY = ball.upperHeight
      self.postCompleted()
