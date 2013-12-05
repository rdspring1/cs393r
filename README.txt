README - CS393R Final Project
Ryan Spring
Edaena Salinas
12/5/2013

----- KickModule Functions -----
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

----- p2Task1 Python Function ----- 
The python code executes a stand command and waits a few seconds before activating the stand balance functions.
In addition, the python code works in conjunction with the c++ code to complete the step action. The stand command in the c++ command
does not execute with the stand balance functions active so the workaround is to execute stand command in the Python code. The c++ stand
balance functions reactivate after the Python stand command executes.
