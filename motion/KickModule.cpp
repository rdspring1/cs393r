#include "KickModule.h"
#include <kinematics/ForwardKinematics.h>
#include <memory/FrameInfoBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/KickModuleBlock.h>
#include <memory/KickParamBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/RobotInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/SpeechBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/WalkRequestBlock.h>
#include <cmath>

#define GRAVITY 9.81f
#define UKP 0.04f
#define UKD 0.02f
#define KP 0.08f
#define KD 0.025f
#define RKP 0.12f
#define RKD 0.025f
#define SMOOTH 0.65
#define WAIT 5
#define XACCEL 0.08f
#define BXACCEL 0.02f
#define YACCEL 0.08f
#define HIPSTAND (DEG_T_RAD * -30.0f)
#define HIPROLL (DEG_T_RAD * -0.26f)
#define ANKLEROLL (DEG_T_RAD * 0.0f)
#define ANKLEPITCH (DEG_T_RAD * 0.0f)
#define MAXANKLE (DEG_T_RAD * 10.0f)
#define MINANKLE (DEG_T_RAD * -10.0f)
#define FORWARDHIP (DEG_T_RAD * -55.0f)
#define BACKWARDHIP (DEG_T_RAD * -25.0f)
#define MAXHIP (DEG_T_RAD * 10.0f)
#define MINHIP (DEG_T_RAD * -10.0f)
#define ABSMAXROLL (DEG_T_RAD * 20.0f)
#define ABSMINROLL (DEG_T_RAD * -20.0f)
#define DETECT 0.02f
#define M_T_MM 1000
#define FRAME_COUNT 100

using namespace std;


KickModule::KickModule() {
}

KickModule::~KickModule() {
}

void KickModule::specifyMemoryDependency() {
	requiresMemoryBlock("frame_info");
	requiresMemoryBlock("processed_joint_commands");
	requiresMemoryBlock("processed_joint_angles");
	requiresMemoryBlock("kick_module");
	requiresMemoryBlock("kick_params");
	requiresMemoryBlock("kick_request");
	requiresMemoryBlock("odometry");
	requiresMemoryBlock("robot_info");
	requiresMemoryBlock("processed_sensors");
	requiresMemoryBlock("speech");
	requiresMemoryBlock("walk_info");
	requiresMemoryBlock("walk_request");
	requiresMemoryBlock("body_model");
}

void KickModule::specifyMemoryBlocks() {
	getMemoryBlock(frame_info_,"frame_info");
	getMemoryBlock(commands_,"processed_joint_commands");
	getMemoryBlock(joint_angles_,"processed_joint_angles");
	getOrAddMemoryBlock(kick_module_,"kick_module");
	getOrAddMemoryBlock(kick_params_,"kick_params");
	getMemoryBlock(kick_request_,"kick_request");
	getMemoryBlock(odometry_,"odometry");
	getMemoryBlock(robot_info_, "robot_info");
	getMemoryBlock(sensors_,"processed_sensors");
	getMemoryBlock(speech_, "speech");
	getMemoryBlock(walk_info_,"walk_info");
	getMemoryBlock(walk_request_,"walk_request");
	getMemoryBlock(body_model_, "body_model");
}

void KickModule::initSpecificModule() {
	for (int i = 0; i < NUM_JOINTS; i++) 
    {
		previous_commands_[i] = joint_angles_->values_[i];
	}
    kick_lock_ = new Lock(Lock::getLockName(memory_,LOCK_KICK), true);

	kick_module_->state_ = KickState::NONE;
	kick_module_->kick_type_ = Kick::NO_KICK;
	kick_module_->swing_leg_ = Kick::LEFT;

	params_normal_ = &kick_params_->params_;
	params_super_ = &kick_params_->params_super_;

    walk_request_->slow_stand_ = true;
	walk_request_->start_balance_ = false;
    walk_request_->exit_step_ = false;

	start_step_ = None;
	balance_count_ = 0;
	l_fsr_front_ = 0; // pressure sensorstepBalances for left foot
	l_fsr_back_ = 0;
	l_fsr_right_ = 0;
	l_fsr_left_ = 0;
	hip_prev_angle_ = HIPSTAND;
	hipframewait = WAIT;
    init_angles_ = false;
    initBalance = false;
    doing_step = false;
    exit_step_count = 0;
}

//used when we enter step balance
void KickModule::resetBalanceValues() {
      hip_prev_angle_ = commands_->angles_[LHipPitch];
      roll_prev_angle_ = commands_->angles_[LHipRoll];
      x_prev_error_ = 0.0;
      y_prev_error_ = 0.0;
}

void KickModule::initJointAngles() {
	if(!init_angles_) 
    {
		for(int joint = 0; joint < NUM_JOINTS; ++joint) {
			commands_->angles_[joint] = joint_angles_->values_[joint];
		}
		for(int joint = 0; joint < NUM_JOINTS; ++joint) {
			previous_commands_[joint] = joint_angles_->values_[joint];
		} 
        init_angles_ = true;
	}
	else 
    {
		for(int joint = 0; joint < NUM_JOINTS; ++joint)
			commands_->angles_[joint] = previous_commands_[joint];
	}

    Vector3<float> c;
    calcCenterOfMass(commands_->angles_, c, true, 0.0f); //gives height of 275
    prev_com_x_ = c.x / 1000; // convert to meters
    prev_com_y_ = c.y / 1000; // convert to meters
 }

void KickModule::getSensedAngles() {
	for(int joint = 0; joint < NUM_JOINTS; ++joint) {
			previous_commands_[joint] = joint_angles_->values_[joint];
		} 
}

void KickModule::updatePreviousAngles() {
	for(int joint = 0; joint < NUM_JOINTS; joint++)
		previous_commands_[joint] = commands_->angles_[joint];
}

bool KickModule::footSensorHasValues() {

    for(int i = fsrLFL; i <= fsrRRR; ++i) {
        if(sensors_->values_[i] <= DETECT)
          return false;
    }
    return true;
}

float KickModule::sumFsrs(FootSensorRegion r) {
	int left_front[2] = {fsrLFL, fsrLFR};
	int left_back[2] = {fsrLRL, fsrLRR};
	int left_left[2] = {fsrLFL, fsrLRL};
	int left_right[2] = {fsrLFR, fsrLRR};    

	int *region;

	if(r == Front)
		region = left_front;
	else if(r == Back)
		region = left_back;
	else if(r == Left)
		region = left_left;
	else if(r == Right)
		region = left_right;

	float sum = 0.0;

	for(int i = 0; i < 2; i++) {
		sum += sensors_->values_[region[i]];
	}
	return sum;
}

void KickModule::initFeetSensorValues() {
	l_fsr_front_ = sumFsrs(Front); // pressure sensors for left foot
	l_fsr_back_ = sumFsrs(Back);
	l_fsr_right_ = sumFsrs(Left);
	l_fsr_left_ = sumFsrs(Right);
}

void KickModule::setHipPitch(float newXAngle)
{
	if(newXAngle < BACKWARDHIP && newXAngle > FORWARDHIP) 
	{
		//cout << "** NewXAngle: " << newXAngle << endl;   
		commands_->angles_[LHipPitch] = newXAngle;
		commands_->angles_[RHipPitch] = newXAngle;  
	}
}

void KickModule::setHipRoll(float newYAngle)
{
    if(newYAngle >= MINHIP && newYAngle <= MAXHIP)
    {
	    //cout << "** NewYAngle: " << newYAngle << endl;   
	    commands_->angles_[LHipRoll] = newYAngle;
        commands_->angles_[RHipRoll] = -1.0f * newYAngle;
    }
}

void KickModule::uprightPitchController()
{
	// HIP PITCH
	float lperror = commands_->angles_[LHipPitch] - HIPSTAND;
	float d_hip_angle = commands_->angles_[LHipPitch] - hip_prev_angle_;
	float upc_xoffset = UKP * sgn(lperror) * min(abs(lperror), 1.0f);
	setHipPitch(commands_->angles_[LHipPitch] - upc_xoffset);
}

void KickModule::uprightRollController()
{
    // HIP ROLL
    float lrerror = commands_->angles_[LHipRoll] - HIPROLL;
	float d_roll_angle = commands_->angles_[LHipRoll] - roll_prev_angle_;
	float upc_yoffset = UKP * sgn(lrerror) * min(abs(lrerror), 1.0f) + UKD * d_roll_angle;
    commands_->angles_[LHipRoll] = commands_->angles_[LHipRoll] - upc_yoffset;
    commands_->angles_[RHipRoll] = -1.0f * (commands_->angles_[LHipRoll] - upc_yoffset);

    float lankle_error = commands_->angles_[LAnkleRoll] - ANKLEROLL;
    float upc_lankle = UKP * sgn(lankle_error) * min(abs(lankle_error), 1.0f);
    commands_->angles_[LAnkleRoll] = commands_->angles_[LAnkleRoll] - upc_lankle;

    float rankle_error = commands_->angles_[RAnkleRoll] - ANKLEROLL;
    float upc_rankle = UKP * sgn(rankle_error) * min(abs(rankle_error), 1.0f);
    commands_->angles_[RAnkleRoll] = commands_->angles_[RAnkleRoll] - upc_rankle;
}

// Joint HipPitch balancing based on feet pressure
void KickModule::footPitchBalance(float x_error, float d_x) 
{
	//cout << d_x << endl;
	float p_x_error = KP * x_error;
	float d_x_error_ = KD * d_x;
    float hac_xoffset = p_x_error + d_x_error_;
	setHipPitch(commands_->angles_[LHipPitch] - hac_xoffset);
}

// Joint HipRoll balancing based on feet pressure
void KickModule::footRollBalance(float y_error, float d_y)
{
    //cout << d_y << endl;
	float p_y_error = RKP * y_error;
	float d_y_error_ = RKD * d_y;
	float hac_yoffset = p_y_error + d_y_error_;
    float newYAngle = commands_->angles_[LHipRoll] - hac_yoffset;
    if(newYAngle >= ABSMINROLL && newYAngle <= ABSMAXROLL)
    {
	    //cout << "** NewYAngle: " << newYAngle << endl;   
	    commands_->angles_[LHipRoll] = max(newYAngle, MINHIP);
	    commands_->angles_[RHipRoll] = min(-1.0f * newYAngle, MAXHIP);

        float newLAnkle = newYAngle - MINHIP;
        if(newLAnkle <= 0.0f)
        {
            commands_->angles_[LAnkleRoll] = max(newLAnkle, MINANKLE);
        }
        
        float newRAnkle = -1.0f * newYAngle - MAXHIP;
        if(newRAnkle >= 0.0f)
        {
            commands_->angles_[RAnkleRoll] = min(newRAnkle, MAXANKLE); 
        }
    }
}

float KickModule::stepBalance() {

    // value for h0, for w = sqrt(g/h0);
    // estimate time remaining for the swing
    // Do the step only if the velocity of the com is high enough, add the check here?

    Vector3<float> c;
    calcCenterOfMass(joint_angles_->values_, c, true, 0.0f); //gives height of 275
    //cout << "******* calcCenterOfMass " << c.x <<", " << c.y << ", " << c.z << endl;

    float h0 = c.z / 1000.0;
    float x0 = c.x / 1000.0; //to meters
    w_ = sqrt(9.81 / h0);
    //cout << "****** x0 prev_com_x " << x0 << " " << prev_com_x_ << endl;
    float v = 0.1; //((x0 - prev_com_x_) / 1000.0) * 100; //convert to meters, we have m/10ms, then convert to seconds
    float c0 = (1.0/2) * (x0 + v / w_);
    float c1 = (1.0/2) * (x0 - v / w_);
    float d0 = (1.0/2) * (w_ * x0 + v);
    float d1 = (1.0/2) * (-1.0 * w_ * x0 + v);
    
    float t = 0.05 - 0.010*step_counter_; //remaining time for the step in seconds CHANGE THIS
    float x = c0 * exp(w_*t) + c1 * exp(-1.0*w_*t); //x position of the center of mass
    float xv = d0 * exp(w_*t) + c1 * exp(-1.0*w_*t);
    
    //cout << "___________com x " << x << endl;
    //cout << "___________com v " << xv << endl;
          
    prev_com_x_ = x0; //update the previous com x
    return xv * sqrt(h0/GRAVITY);
}

//balance process frame
void KickModule::processFrame() 
{
    if( walk_request_->exit_step_) 
    {
        resetBalanceValues();
        getSensedAngles();
        return;    
    }
   
	if(walk_request_->start_balance_ && !doing_step) 
    {
        initBalance = true;

	    // STIFFNESS
	    // Maintain the stiffness from now on after we turn off walk   
	    walk_request_->noWalk();
	    initJointAngles();
	    initFeetSensorValues();

        // Position and Velocity - Pressure Sensors for Left Foot
        l_fsr_front_ = (1 - SMOOTH) * l_fsr_front_ + SMOOTH * sumFsrs(Front);
        l_fsr_back_ = (1 - SMOOTH) * l_fsr_back_ + SMOOTH * sumFsrs(Back);
        l_fsr_left_ = (1 - SMOOTH) * l_fsr_left_ + SMOOTH * sumFsrs(Left);
        l_fsr_right_ = (1 - SMOOTH) * l_fsr_right_ + SMOOTH * sumFsrs(Right);

        float x_error = l_fsr_front_ - l_fsr_back_;
        float y_error = l_fsr_right_ - l_fsr_left_;

        float d_x = x_error - x_prev_error_;
        float d_y = y_error - y_prev_error_;

        bool front = sumFsrs(Front) < DETECT;
        bool back = sumFsrs(Back) < DETECT;
        bool left =  sumFsrs(Left) < DETECT;
        bool right =  sumFsrs(Right) < DETECT;

        //bool tr = sensors_->values_[fsrLFR] < DETECT;
        //bool tl = sensors_->values_[fsrLFL] < DETECT;
        //bool br = sensors_->values_[fsrLRR] < DETECT;
        //bool bl = sensors_->values_[fsrLRL] < DETECT;
        //bool all = tr | tl | br | bl;

        if(abs(d_x) > BXACCEL && front)
        {   
            cout << "Start Back Kick Step" << endl;
            if(kick_lock_->try_lock())
            {   
                // Execute Backward Step
                cout << walk_request_->start_balance_ << " ____back step " << frame_info_->frame_id << endl;
                start_step_ = Back;
                walk_request_->start_balance_ = false; ///eb
                kickParamsGenerator(params_normal_, 50, false, true); ///eb
                kick_request_->set(Kick::STRAIGHT, Kick::RIGHT, 0, 100); ///eb
                processFrameForStep(); ///eb
                doing_step = true; ///eb
                kick_lock_->unlock();
                return;
            }
        }
    	
        if(hipframewait == 0) 
        {   
	        if(abs(d_x) > XACCEL)
            {
                footPitchBalance(x_error, d_x);
            }
            else
            {
                uprightPitchController();
            }
 
            if(abs(d_y) > YACCEL)
            {
                footRollBalance(y_error, d_y);
            }
            else
            {
                uprightRollController();
            }

            hipframewait = WAIT;
        }

        if(hipframewait > 0)
        {
            --hipframewait;
        }
                  
        hip_prev_angle_ = commands_->angles_[LHipPitch];
        roll_prev_angle_ = commands_->angles_[LHipRoll];
        x_prev_error_ = x_error;
        y_prev_error_ = y_error;
        
	    initStiffness();
	    commands_->stiffness_time_ = 40;
        updatePreviousAngles();  
	    commands_->send_body_angles_ = true;
	    commands_->body_angle_time_ = 100; // this changes depending on the movement
	}//if start balance

    if(!walk_request_->start_balance_ && doing_step) 
    {
        processFrameForStep();
    } // if processStep
} //process

void KickModule::processFrameForStep() {
  processKickRequest();

  //cout << "processFramFor() " << KickState::getName(kick_module_->state_) << endl;

  if (kick_module_->state_ == KickState::STAND) {
    kick_request_->kick_running_ = true;
    // only transition into kick if we think we're stable
    //std::cout << "walk_info_->instability_: " << walk_info_->instability_ << std::endl;  
    cout << "____WALK IS ACTIVE " << walk_info_->walk_is_active_ << endl;  
    if ((!walk_info_->walk_is_active_)) {
        ///cout << "____ 2 " << endl;  
      if (getFramesInState() >= state_params_->state_time / 10) { // divide by 10 to convert from ms to frames.
        walk_request_->noWalk();
        ///cout << "____ 3 " << endl; 
        transitionToState((KickState::State)(kick_module_->state_ + 1));
      } else {
        ///cout << "____ 4 " << endl; 
        walk_request_->stand();
      }
    } else {
        ///cout << "____ 5 " << endl; 
      walk_request_->stand();
      transitionToState(KickState::STAND); // restart this state
      return;
    }
  }

  if ((kick_module_->state_ == KickState::WALK) && (frame_info_->seconds_since_start > kick_module_->state_start_time_ + 0.5)) {
    ///cout << "____ 6 " << endl; 
    startKick();
  }

  // handle transitions
  while ((kick_module_->state_ != KickState::NONE) && (getFramesInState() >= state_params_->state_time / 10)) { // divide by 10 to convert from ms to frames.
    ///cout << "____ 7 " << endl; 
    if (kick_module_->state_ == KickState::WALK)
      break;
    transitionToState((KickState::State)(kick_module_->state_ + 1));
  }
  
  if (kick_module_->state_ == KickState::NONE) {
    //cout << "____ 8 " << endl; 
    // not in a kick, let vision know
    kick_request_->kick_running_ = false;
    kick_module_->kick_type_ = Kick::NO_KICK;
  } else {
    // if we're still in a kick, do it
    kick();
    ///cout << "____ 9 " << endl; 
    setKickOdometry();
    kick_request_->kick_running_ = true;
  }
}

int KickModule::getFramesInState() {
  return frame_info_->frame_id - kick_module_->state_start_frame_;
}

float KickModule::getTimeInState() {
	return frame_info_->seconds_since_start - kick_module_->state_start_time_;
}

float KickModule::getMillisecondsInState() {
	return 1000.0f * (frame_info_->seconds_since_start - kick_module_->state_start_time_);
}

float KickModule::getFramesPerSecond() {
	return 1.0f * getFramesInState() / getTimeInState();
}

void KickModule::processKickRequest() {
    //cout << "____processKickRequ() " << kick_request_->kick_type_ << " " << KickState::getName(kick_module_->state_) << endl;
	if (kick_request_->kick_type_ == Kick::ABORT) {
		kick_module_->state_ = KickState::NONE;
        /// cout << "______1_______ processKickRequest()  " << endl;
	} else if ((kick_request_->kick_type_ != Kick::NO_KICK) && (kick_module_->state_ == KickState::NONE)) {
        ///cout << "______2_______ processKickRequest()  " << endl;
		startKick();
	}
}

KickParameters* KickModule::kickParamsGenerator(KickParameters * kp, float distance, bool forward, bool rightleg)
{
    // times in milliseconds
    float liftAmount = 40; 
    float backAmount = (forward) ? abs(distance) : abs(distance) * -1.0f;
    float throughAmount = 100;
    float liftAlignAmount = 50;
    float liftKickAmount = 0;
    float comHeight = 175;
    float comOffset = (rightleg) ? 10 : -10;
    float comOffsetX = (forward) ? -20 : 20;
    float kick_time = 0;
    KickStateInfo* info = NULL;

    info = kp->getStateInfoPtr(KickState::STAND);
    info->state_time = 200;
    info->joint_time = 200;
    info->com = Vector3<float>(comOffsetX,50,comHeight);
    info->swing = Vector3<float>(0,110,0);
    kick_time += info->state_time;

    info = kp->getStateInfoPtr(KickState::SHIFT);
    info->state_time = 150;
    info->joint_time = 150;
    info->com = Vector3<float>(comOffsetX,comOffset + 30,comHeight);
    info->swing = Vector3<float>(0,110,0);
    kick_time += info->state_time;

    info = kp->getStateInfoPtr(KickState::LIFT);
    info->state_time = 50;
    info->joint_time = 150;
    info->com = Vector3<float>(comOffsetX,comOffset,comHeight);
    info->swing = Vector3<float>(0,110,10);
    kick_time += info->state_time;

    info = kp->getStateInfoPtr(KickState::ALIGN);
    info->state_time = 150;
    info->joint_time = 150;
    info->com = Vector3<float>(comOffsetX,comOffset,comHeight);
    info->swing = Vector3<float>(backAmount,110,liftAlignAmount);
    kick_time += info->state_time;

    info = kp->getStateInfoPtr(KickState::KICK1);
    info->state_time = 0;
    info->joint_time = 50;
    info->com = Vector3<float>(comOffsetX,comOffset,comHeight);
    info->swing = Vector3<float>(0,110,(liftKickAmount+liftAlignAmount)/2);
    kick_time += info->state_time;

    info = kp->getStateInfoPtr(KickState::KICK2);
    info->state_time = 0;
    info->joint_time = 50;
    info->com = Vector3<float>(comOffsetX,comOffset,comHeight);
    info->swing = Vector3<float>(backAmount,110,liftKickAmount);
    kick_time += info->state_time;

    info = kp->getStateInfoPtr(KickState::FOOTDOWN);
    info->state_time = 0;
    info->joint_time = 300;
    info->com = Vector3<float>(comOffsetX,comOffset,comHeight);
    info->swing = Vector3<float>(0, 110,liftAmount/2);
    kick_time += info->state_time;

    info = kp->getStateInfoPtr(KickState::SHIFTBACK);
    info->state_time = 0;
    info->joint_time = 500;
    info->com = Vector3<float>(comOffsetX,comOffset,comHeight);
    info->swing = Vector3<float>(0,110,0);
    kick_time += info->state_time;

    info = kp->getStateInfoPtr(KickState::FINISHSTAND);
    info->state_time = 8000;
    info->joint_time = 500;
    info->com = Vector3<float>(comOffsetX/2,50,comHeight);
    info->swing = Vector3<float>(backAmount,110,0);
    kick_time += info->state_time;

    // SPLINE STATE
    info = kp->getStateInfoPtr(KickState::SPLINE);
    info->state_time = 100;
    info->joint_time = info->state_time;
    info->com = Vector3<float>(comOffsetX,comOffset,comHeight);
    info->swing = Vector3<float>(0,110,0);
    kick_time += info->state_time;
    return kp;
}

void KickModule::startKick() {
	kick_module_->kick_type_ = kick_request_->kick_type_;
	kick_module_->swing_leg_ = kick_request_->kick_leg_;
	kick_module_->set_kick_odometry_ = false;
	kick_module_->desired_kick_distance_ = kick_request_->desired_distance_;
	kick_module_->desired_kick_angle_ = kick_request_->desired_angle_;
	params_ = params_normal_; // do normal kick

	invalidCount = 0;
	initStiffness();
	if (params_->step_into_kick_)
		transitionToState(KickState::SHIFT);
	else
		transitionToState(KickState::STAND);
	kick_module_->kick_start_time_ = frame_info_->seconds_since_start;
	//std::cout << "Kick requested: " <<  kick_module_->kick_type_ << "  requested distance: " << kick_module_->desired_kick_distance_ << "  requested angle: " << kick_module_->desired_kick_angle_ << std::endl;

	/// setHead();
}

void KickModule::setHead() {
	if ((kick_module_->state_ == KickState::STAND) || (kick_module_->state_ == KickState::SHIFT) || (kick_module_->state_ == KickState::LIFT) || (kick_module_->state_ == KickState::ALIGN) || (kick_module_->state_ == KickState::SPLINE)) {
		commands_->setHeadPan(DEG_T_RAD*0, 0.1, false);
		commands_->setHeadTilt(DEG_T_RAD*-21, 0.1, false);
	}
}

void KickModule::initStiffness() {
	for (int i = 0; i < NUM_JOINTS; i++)
		commands_->stiffness_[i] = 1.0;
	commands_->send_stiffness_ = true;
	commands_->stiffness_time_ = 30;
}

void KickModule::setLegStiffness(float stiff) {
	for (int i = 2; i < 14; i++)
		commands_->stiffness_[i] = stiff;
	commands_->send_stiffness_ = true;
	commands_->stiffness_time_ = 30;
}

void KickModule::transitionToState(KickState::State state) {
    std::cout << "___ transitionToState: " << KickState::getName(kick_module_->state_) << " -> " << KickState::getName(state) << " " << frame_info_->frame_id << std::endl;
    if(initBalance && kick_module_->state_ == KickState::FINISHSTAND && state == KickState::NONE)
    {
        cout << "Exit Kick Step" << endl;
        start_step_ = None;
        doing_step = false;
        resetBalanceValues(); ///eb
        walk_request_->exit_step_ = true; ///eb
        //exit_step_ = true;
    }

	if (kick_module_->state_ == KickState::STAND && !walk_info_->walk_is_active_) {
		bool continue_kick = handleAiming();
		if (continue_kick) {
			invalidCount = 0;
		} else {
			invalidCount = invalidCount + 1;
		}
		if (invalidCount > 10 || (state != KickState::STAND && invalidCount > 5)) {
			kick_module_->state_ = KickState::NONE;
			std::cout << "Abandon kick - invalid count too high" << std::endl;
			return;
		}
	}

	if (state == KickState::SHIFT) { 
		if (params_->states[KickState::SPLINE].state_time > 0) {
			calcSwingSplinePts(); ///eb
		}
	}

	kick_module_->state_ = state;
	kick_module_->sent_command_ = false;
	kick_module_->sent_steady_state_command_ = false;
	kick_module_->state_start_time_ = frame_info_->seconds_since_start;
	kick_module_->state_start_frame_ = frame_info_->frame_id;
	state_params_ = &(params_->states[kick_module_->state_]);
}

bool KickModule::chooseKickLeg() {
	// choose a leg if switchable
	if (kick_module_->swing_leg_ == Kick::SWITCHABLE) {
		if (kick_request_->ball_rel_y_ > 0) {
			kick_module_->swing_leg_ = Kick::LEFT;
		} else {
			kick_module_->swing_leg_ = Kick::RIGHT;
		}
	}
	return true;
}

bool KickModule::checkKickValidity() {
	// std::cout << "Kick request - forward: " << kick_request_->ball_rel_x_ << ", side: " << kick_request_->ball_rel_y_ << ", dist side: " << kick_module_->ball_dist_side_ << std::endl; 
	if((kick_module_->swing_leg_ == Kick::LEFT && kick_module_->ball_dist_side_ > 30) || (kick_module_->swing_leg_ == Kick::RIGHT && kick_module_->ball_dist_side_ < -50)) {
		std::cout << "Ball too far sideways: " << kick_module_->ball_dist_side_ << std::endl;
		return false;
	}
	//forward
	if (kick_module_->ball_dist_forward_ > 70) {
		std::cout << "Ball too far forwards: " << kick_module_->ball_dist_forward_ << std::endl;
		return false;
	}
	return true;
}

bool KickModule::handleAiming() {
	if (!kick_request_->ball_seen_) {
		std::cout << "Ball not seen." << std::endl;
		return false;
	}
	if (!chooseKickLeg()) {
		return false;
	}
	calcBallPosWRTSwingLeg();
	if (!checkKickValidity()) {
		return false;
	}
	return true;
}

void KickModule::kick() {
  Vector3<float> com = state_params_->com;

  if (kick_module_->state_ == KickState::SPLINE) {
    sendSplineCOMCommands(com); ///eb
    return;
  }

  bool send_commands = false;
  Vector3<float> swing;
  bool move_com = true;
  float command_time = state_params_->joint_time;

  if (!kick_module_->sent_command_) {
    send_commands = true;
    swing = state_params_->swing;
    kick_module_->sent_command_ = true;
  } else if ((!kick_module_->sent_steady_state_command_) && (getMillisecondsInState() > command_time)) {
    send_commands = true;
    swing = state_params_->swing;
    command_time = state_params_->state_time - getMillisecondsInState();
    kick_module_->sent_steady_state_command_ = true;
  }

  if (send_commands) {
    /// setHead();
    bool is_left_swing = (kick_module_->swing_leg_ == Kick::LEFT);
    int dir = 1;
    if (!is_left_swing)
      dir = -1;
  
    if (kick_module_->state_ == KickState::ALIGN) {
      Vector3<float> temp;
      getSwingTargets(swing,temp);
    } 

    com.y *= dir;
    swing.y *= dir;
  
    Pose3D swing_target(swing);
    float roll = DEG_T_RAD * 0;

   // cout << "___kick() swing_target " << swing_target.translation.x << ", " << swing_target.translation.y << ", " << swing_target.translation.z << endl;
    //cout << "___kick() com " << com.x << ", " << com.y << ", " << com.z << endl;
    calcJointTargets(com,swing_target,is_left_swing,commands_->angles_,move_com,roll);
  
    commands_->send_body_angles_ = true;
    commands_->body_angle_time_ = command_time;
  } else {
    commands_->send_body_angles_ = false;
  }
}


void KickModule::calcSwingSplinePts() {
    Vector3<float> align = params_->states[KickState::ALIGN].swing;
    Vector3<float> kick = params_->states[KickState::KICK2].swing;

    getSwingTargets(align,kick);

    double time = 0;  //default
    if (kick_module_->swing_leg_ == Kick::RIGHT) {
    time = -0.0698*kick_module_->desired_kick_distance_ + 399.4; // tuned on Alison
    } else {
    time = -0.0685*kick_module_->desired_kick_distance_ + 382.5;
    }
    printf("desired: %2.f\n", kick_module_->desired_kick_distance_);
    //crop the value to within minval, maxval
    //crop(val, minval, maxval)
    //time = crop(time,200,400); ///eb
    time = 200;
    std::cout << "time: " << time << std::endl; 
    //cout<<"Time for kick"<<time<<endl;

    int num_pts = 5;
    params_->states[KickState::SPLINE].state_time = time;
    params_->states[KickState::SPLINE].joint_time = time;
    double timesInMs[] = {0,10,20,time-10,time};
    double xs[] = {align.x,align.x,align.x,kick.x,kick.x};
    double ys[] = {align.y,align.y,align.y,kick.y,kick.y};
    double zs[] = {align.z,align.z,align.z,kick.z,kick.z};
    setSwingSpline(num_pts,timesInMs,xs,ys,zs);

    cout << "calcSwingSplinepts() align " << align.x << ", " << align.y << ", " << align.z << endl;
    cout << "calcSwingSplinepts() kick " << kick.x << ", " << kick.y << ", " << kick.z << endl;

}

void KickModule::setSwingSpline(int num_pts,double timesInMs[], double xs[], double ys[], double zs[]) {
	swing_spline_.set(num_pts,timesInMs,xs,ys,zs,true);
}

void KickModule::sendSplineCOMCommands(const Vector3<float> &com_in) {
    float time = getMillisecondsInState();
    Vector3<float> swing;
    bool is_left_swing = (kick_module_->swing_leg_ == Kick::LEFT);
    int dir = 1;
    Vector3<float> com(com_in);
    if (!is_left_swing)
    dir = -1;

    swing_spline_.calc(time,swing);
    swing.y *= dir;
    com.y *= dir;

    Pose3D swing_target(swing);

    calcJointTargets(com,swing_target,is_left_swing,commands_->angles_,true,0);
    commands_->send_body_angles_ = true;
    commands_->body_angle_time_ = 10;
}

void KickModule::getSwingTargets(Vector3<float> &align, Vector3<float> &kick) {
	float ideal_ball_side_left_swing_ = params_->ideal_ball_side_left_swing_;
	float ideal_ball_side_right_swing_ = params_->ideal_ball_side_right_swing_;
	if (kick_module_->swing_leg_ == Kick::RIGHT) {
		align.y = align.y - (kick_module_->ball_dist_side_ - ideal_ball_side_right_swing_);
		kick.y = kick.y - (kick_module_->ball_dist_side_ - ideal_ball_side_right_swing_);
	} else {
		align.y = align.y + (kick_module_->ball_dist_side_ - ideal_ball_side_left_swing_);
		kick.y = kick.y + (kick_module_->ball_dist_side_ - ideal_ball_side_left_swing_);
	}
    cout << "__getSwingTargets() align.y kick.y " << align.y << ", " << kick.y << endl;
}

void KickModule::calcBallPosWRTSwingLeg() {
	int offset = 50;
	if (kick_module_->swing_leg_ == Kick::LEFT) {
		kick_module_->ball_dist_side_ = kick_request_->ball_rel_y_ - offset;
	} else {
		kick_module_->ball_dist_side_ = kick_request_->ball_rel_y_ + offset;
	}
	kick_module_->ball_dist_forward_ = kick_request_->ball_rel_x_;
	//std::cout << "rel side: " << kick_request_->ball_rel_y_ << ", rel forward: " << kick_request_->ball_rel_x_ << std::endl;
}

void KickModule::setKickOdometry() {
	if (!kick_module_->set_kick_odometry_ && (kick_module_->state_ > KickState::SPLINE) && (kick_module_->state_ != KickState::WALK)) {
		kick_module_->set_kick_odometry_ = true;
		odometry_->didKick = true;
		odometry_->kickVelocity = kick_module_->desired_kick_distance_ / 1.2; // TODO: make this more intelligent
		odometry_->kickHeading = kick_module_->desired_kick_angle_;
	}
}

void KickModule::calcJointTargets(const Vector3<float> &com_target, const Pose3D &swing_rel_stance, bool is_left_swing, float command_angles[NUM_JOINTS], bool move_com, float roll) {
	if (is_left_swing)
		roll *= 1;


    //cout << endl << "calcJointTar() swing_rel_stance " << swing_rel_stance << endl;
	// calculate body model from last commands
	Vector3<float> com;
	calcCenterOfMass(previous_commands_, com, !is_left_swing,0.0f);

	// figure out which leg is stance && which is swing
	Pose3D left_target;
	Pose3D right_target;
	Pose3D *stance_target = &right_target;
	Pose3D *swing_target = &left_target;

	BodyPart::Part stance_foot = BodyPart::right_foot;
	//BodyPart::Part swing_foot = BodyPart::left_foot;
	//int stance_hip_roll = RHipRoll;

	if (!is_left_swing) {
		stance_target = &(left_target);
		swing_target = &(right_target);
		stance_foot = BodyPart::left_foot;
		//swing_foot = BodyPart::right_foot;
		//stance_hip_roll = LHipRoll;
	}

	// the offset from stance leg to torso
	Vector3<float> stance_to_torso_offset;
	stance_to_torso_offset = command_body_model_.abs_parts_[stance_foot].translation - command_body_model_.abs_parts_[BodyPart::torso].translation;

	Vector3<float> abs_desired_com;
	abs_desired_com.x = com_target.x + stance_to_torso_offset.x;
	abs_desired_com.y = com_target.y + stance_to_torso_offset.y;
	abs_desired_com.z = com_target.z;


	//cout << "_____COM ABS DESIRED " << abs_desired_com.x << ", " << abs_desired_com.y << ", " << abs_desired_com.z << endl;

	// stance leg starts out at current position
	// && will be offset later by com change we want
	stance_target->rotation = RotationMatrix(0,0,0);
	stance_target->translation = command_body_model_.abs_parts_[stance_foot].translation;
	stance_target->rotation.rotateZ(command_body_model_.abs_parts_[stance_foot].rotation.getZAngle());


	// convert from stance in abs frame to stance in torso frame
	stance_target->translation -= command_body_model_.abs_parts_[BodyPart::torso].translation;
	stance_target->translation.x = -com_target.x;
	stance_target->translation.z = -com_target.z;

	// convert swing leg from stance to torso frame
	*swing_target = swing_rel_stance;
	Vector2<float> swingXY;
	swingXY.x = swing_target->translation.x;
	swingXY.y = swing_target->translation.y;
	swingXY.rotate(stance_target->rotation.getZAngle());
	swing_target->translation.x = swingXY.x;
	swing_target->translation.y = swingXY.y;
    //cout << "___ swing_target y before adding stance_target " << swing_target->translation.y << " + " << stance_target->translation.y<< endl;
	swing_target->translation += stance_target->translation;
	swing_target->rotation.rotateZ(stance_target->rotation.getZAngle());

	Vector3<float> com_err;
	float max_acceptable_com_err = 0.25; //1.0;
	for (int i = 0; i < 100; i++) {
		com_err = abs_desired_com - com;
		if (!move_com)
			com_err = Vector3<float>(0,0,0);
		//if (com_err.abs() < max_acceptable_com_err && i > 0)
		if (fabs(com_err.y) < max_acceptable_com_err && i > 0)
			break;

		// so we don't wildly overshoot 
		// (z movements kind of result in double com movements... torso lowers
		// && we lower our weight)
		com_err.z *= 0.5;

		// get stance foot position relative to torso
		// move stance foot opposite direction from com error
		//stance_target->translation.x -= com_err.x;
		stance_target->translation.y -= com_err.y;
		//stance_target->translation.z -= com_err.z;

		// also move desired com by this amount...
		// since we moved stance leg
		//abs_desired_com.x -= com_err.x;
		abs_desired_com.y -= com_err.y;

		// Todd: desired com z is absolute now
		//abs_desired_com.z -= pen_err.z;

		// as stance leg moves one way, we have to move the swing leg relative to
		// the torso to match it
		// so swing leg to stance leg distance remains the same
		//swing_target->translation.x -= com_err.x;
		swing_target->translation.y -= com_err.y;
		//swing_target->translation.z -= com_err.z;

		if (stance_target->translation.z < -203) stance_target->translation.z = -203;
		if (swing_target->translation.z < -203) swing_target->translation.z = -203;

		bool left_compliant  = true;//false;
		bool right_compliant = true;//false;

		// calculate tilt roll from stance leg
		TiltRoll tr = ForwardKinematics::calculateTiltRollFromLeg(!is_left_swing, commands_->angles_, robot_info_->dimensions_);
		tr.roll_ = roll;

		commandLegsRelativeToTorso(command_angles, left_target, right_target, tr.tilt_, tr.roll_, 0.0f, left_compliant,right_compliant);
		setArms(command_angles);

		// calculate new com based on these joint commands
		calcCenterOfMass(command_angles, com, !is_left_swing, 0.0f);
	} 

	for (int i = 0; i < NUM_JOINTS; i++) {
		previous_commands_[i] = commands_->angles_[i];
	}
}

void KickModule::setArms(float command_angles[NUM_JOINTS]) {
	if (walk_request_->keep_arms_out_) {
		command_angles[LShoulderPitch] = DEG_T_RAD * -116;
		command_angles[LShoulderRoll] = DEG_T_RAD * 12;
		command_angles[LElbowYaw] = DEG_T_RAD * -85;
		command_angles[LElbowRoll] = DEG_T_RAD * -0;
		command_angles[RShoulderPitch] = DEG_T_RAD * -116;
		command_angles[RShoulderRoll] = DEG_T_RAD * 12;
		command_angles[RElbowYaw] = DEG_T_RAD * -85;
		command_angles[RElbowRoll] = DEG_T_RAD * -0;
	} else {
		command_angles[LShoulderPitch] = DEG_T_RAD*-116;
		command_angles[RShoulderPitch] = DEG_T_RAD*-116;

		command_angles[LShoulderRoll] = DEG_T_RAD*8;
		command_angles[RShoulderRoll] = DEG_T_RAD*8;

		command_angles[LElbowRoll] = DEG_T_RAD*-53;
		command_angles[RElbowRoll] = DEG_T_RAD*-53;

		command_angles[LElbowYaw] = DEG_T_RAD*25;
		command_angles[RElbowYaw] = DEG_T_RAD*25;
	}
}

void KickModule::calcCenterOfMass(float *command_angles, Vector3<float> &center_of_mass, bool stance_is_left, float tilt_roll_factor){

	// calculate tilt roll from stance leg
	TiltRoll tr = ForwardKinematics::calculateTiltRollFromLeg(stance_is_left, command_angles, robot_info_->dimensions_);

	// use some fraction of tilt && roll
	tr.tilt_ *= tilt_roll_factor;
	tr.roll_ *= tilt_roll_factor;

	ForwardKinematics::calculateRelativePose(command_angles, tr.tilt_, tr.roll_, command_body_model_.rel_parts_, robot_info_->dimensions_.values_);
	Pose3D base = ForwardKinematics::calculateVirtualBase(stance_is_left, command_body_model_.rel_parts_);
	ForwardKinematics::calculateAbsolutePose(base, command_body_model_.rel_parts_, command_body_model_.abs_parts_);
	ForwardKinematics::calculateCoM(command_body_model_.abs_parts_, command_body_model_.center_of_mass_, robot_info_->mass_calibration_);
	center_of_mass = command_body_model_.center_of_mass_;

	//cout << "____ calcCenterOfMass " << center_of_mass.x << ", " << center_of_mass.y << ", " << center_of_mass.z << endl;
}

void KickModule::commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float /*tilt*/, float roll, float /*tilt_roll_factor*/, bool left_compliant, bool right_compliant) {

	RotationMatrix rot;
	RotationMatrix foot_rotation;
	rot.rotateX(roll);
	foot_rotation.rotateX(-roll);
	if (left_compliant) {
		left_target.translation = rot * left_target.translation;
		left_target.rotation = left_target.rotation * foot_rotation;
	}
	if (right_compliant) {
		right_target.translation = rot * right_target.translation;
		right_target.rotation = right_target.rotation * foot_rotation;
	}
	bool isValid = false;

	//cout << "*** command legs relative to torso " << endl;
	cout << "commandLegsRel() left foot x y z " << left_target.translation.x << ", " << left_target.translation.y << ", " << left_target.translation.z << endl;
	//cout << "left foot rotation x y z " << left_target.rotation.getXAngle() << ", " << left_target.rotation.getYAngle() << ", " << left_target.rotation.getZAngle() << endl;
	cout << "commandLegsRel() right foot x y z " << right_target.translation.x << ", " << right_target.translation.y << ", " << right_target.translation.z << endl;
	//cout << "right foot rotation x y z " << right_target.rotation.getXAngle() << ", " << right_target.rotation.getXAngle() << ", " << right_target.rotation.getXAngle() << endl;

	isValid = inverse_kinematics_.calcLegJoints(left_target, right_target, command_angles, robot_info_->dimensions_);


	command_angles[LHipYawPitch] = 0.0;
	command_angles[RHipYawPitch] = 0.0;



}
