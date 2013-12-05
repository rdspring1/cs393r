#ifndef KICKMODULE
#define KICKMODULE

#include <memory/Memory.h>
#include <Module.h>
#include <common/RobotDimensions.h>
#include <common/MassCalibration.h>
#include <kinematics/InverseKinematics.h>
#include <motion/KickParameters.h>
#include <common/RingBufferWithSum.h>
#include <memory/BodyModelBlock.h>
#include <memory/Lock.h>
#include <math/Spline3D.h>
#define NUM_STEP_POSITIONS 20

class FrameInfoBlock;
class JointBlock;
class JointCommandBlock;
class KickModuleBlock;
class KickParamBlock;
class KickRequestBlock;
class OdometryBlock;
class RobotInfoBlock;
class SensorBlock;
class SpeechBlock;
class WalkInfoBlock;
class WalkRequestBlock;

enum FootSensorRegion 
{
    None,
	Front,
	Back,
	Left,
	Right,
};

class KickModule : public Module {
	public:
		KickModule ();
		virtual ~KickModule ();
		void specifyMemoryDependency();
		void specifyMemoryBlocks();
		void initSpecificModule();
		void processFrame();
		void setStand(const float *stand_angles) 
        {
			for (int i = 0; i < NUM_JOINTS; i++)
				STAND_ANGLES[i] = stand_angles[i];
		}

	private:
        template <typename T> int sgn(T val)
        {
            return (T(0) < val) - (val < T(0));
        }
		int getFramesInState();
		float getTimeInState();
		float getMillisecondsInState();
		float getFramesPerSecond();
		void processKickRequest();
		void startKick();
		void setHead();
		void initStiffness();
		void setLegStiffness(float stiff);
		void transitionToState(KickState::State state);
		bool chooseKickLeg();
		bool checkKickValidity();
		bool handleAiming();
		void kick();
		void calcSwingSplinePts();
		void setSwingSpline(int num_pts,double timesInMs[], double xs[], double ys[], double zs[]);
		void sendSplineCOMCommands(const Vector3<float> &com_target);
		void getSwingTargets(Vector3<float> &align, Vector3<float> &kick);
		void calcBallPosWRTSwingLeg();
		void setKickOdometry();
		void calcJointTargets(const Vector3<float> &com_target, const Pose3D &swing_rel_stance, bool left_swing, float command_angles[NUM_JOINTS],bool move_com,float roll);
		void setArms(float command_angles[NUM_JOINTS]);
		void calcCenterOfMass(float *command_angles, Vector3<float> &center_of_mass, bool stance_is_left, float tilt_roll_factor);
		void commandLegsRelativeToTorso(float *command_angles, Pose3D left_target, Pose3D right_target, float tilt, float roll, float tilt_roll_factor, bool left_compliant, bool right_compliant);
        
        /// Initializes the joint angles used to balance the robot
		void initJointAngles();
        /// Initializes the pressure values for each of the 4 foot regions - front, back, left, right
		void initFeetSensorValues();
        /// Sums the pressure values of the given foot region on the left foot
        /// If the given region is Front, the function will add the value of the top left and top right pressure sensors of the foot
		float sumFsrs(FootSensorRegion r);
        /// Determine if any of the foot sensors are not on the ground
        bool footSensorHasValues();
        /// Set the Left and Right Hip Pitch joint angles to a valid angle
        void setHipPitch(float newXAngle);
        /// Set the Left and Right Hip Roll joint angles to a valid angle
        void setHipRoll(float newYAngle);
        /// Returns the Hip Pitch Joints to their regular stand values using PID control
        void uprightPitchController();
        /// Returns the Hip Roll and Ankle Roll Joints to their regular stand values using PID control
        void uprightRollController();
        /// Modify Hip Pitch Joint Angles to balance the robot based on feet pressure
        void footPitchBalance(float x_error, float d_x);
        /// Modify Hip Roll and Ankle Roll Joint Angles to balance the robot based on feet pressure
        void footRollBalance(float y_error, float d_y);
        /// A Process Frame function used to execute a step
        void processFrameForStep();
        /// A function that dynamically generates a step for the robot alternating the center of mass depending on the type of step
        KickParameters* kickParamsGenerator(KickParameters * kp, float distance, bool forward, bool rightleg);
        /// A function that resets the Hip and Ankle controllers after executing a step
        void resetBalanceValues(); 
        /// Determine the Joint Angles from the Sensor
        void getSensedAngles();
        /// Update the previous joint angles with the command angles for the current process frame
        void updatePreviousAngles();
        /// Generates the size of the step to prevent the robot from following using the position and velocity of the center of mass
        float stepBalance();

	private:
		float previous_commands_[NUM_JOINTS];
		float lfoot_x_[NUM_STEP_POSITIONS]; // step positions for each foot
		float rfoot_x_[NUM_STEP_POSITIONS];
		float lfoot_y_[NUM_STEP_POSITIONS];
		float rfoot_y_[NUM_STEP_POSITIONS];
		float lfoot_z_[NUM_STEP_POSITIONS];
		float rfoot_z_[NUM_STEP_POSITIONS];
		float l_fsr_front_; // pressure sensors for left foot
		float l_fsr_back_;
		float l_fsr_right_;
		float l_fsr_left_;
		float x_prev_error_;
		float y_prev_error_;
		float hip_prev_angle_;
        float roll_prev_angle_;
		float hipframewait;
        float w_;
        float prev_com_x_;
        float prev_com_y_;
        int step_counter_;
        bool init_angles_;
        bool stop; ///temp added for kick
        bool set; ///temp added for kick
        bool initBalance;

	public:
		Spline3D swing_spline_;
		Spline3D stance_spline_;
        Spline3D step_spline_;
		int invalidCount;

	private:
		InverseKinematics inverse_kinematics_;
		float STAND_ANGLES[NUM_JOINTS];

	private:
		KickStateInfo *state_params_;
		KickParameters *params_;
		KickParameters *params_normal_;
		KickParameters *params_super_;
		BodyModelBlock command_body_model_; // for local computation about commands
		BodyModelBlock *body_model_;
		Vector3<float> prev_center_of_mass_;

		FrameInfoBlock *frame_info_;
		JointCommandBlock *commands_;
		JointBlock *joint_angles_;
		KickModuleBlock *kick_module_;
		KickParamBlock *kick_params_;
		KickRequestBlock *kick_request_;
		OdometryBlock *odometry_;
		RobotInfoBlock *robot_info_;
		SensorBlock *sensors_;
		SpeechBlock *speech_;
		WalkInfoBlock *walk_info_;
		WalkRequestBlock *walk_request_;


		FootSensorRegion start_step_;
		int balance_count_;
        bool doing_step;
        Lock* kick_lock_;
        bool exit_step_;
        int exit_step_count;
};

#endif
