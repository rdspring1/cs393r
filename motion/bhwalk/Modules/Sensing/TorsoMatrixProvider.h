/**
* @file TorsoMatrixProvider.h
* Declaration of module TorsoMatrixProvider.
* @author Colin Graf
*/

#pragma once

//#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/DamageConfiguration.h"

//MODULE(TorsoMatrixProvider)
//  REQUIRES(FilteredSensorData)
//  REQUIRES(RobotModel)
//  REQUIRES(RobotDimensionsBH)
//  REQUIRES(GroundContactState)
//  REQUIRES(DamageConfiguration)
//  PROVIDES(TorsoMatrixBH)
//  USES(TorsoMatrixBH)
//  PROVIDES_WITH_MODIFY_AND_OUTPUT(OdometryData)
//END_MODULE

/**
* @class TorsoMatrixProvider
* A module that provides the (estimated) position and velocity of the inertia board.
*/
class TorsoMatrixProvider //: public TorsoMatrixProviderBase
{
private:
  float lastLeftFootZRotation; /**< The last z-rotation of the left foot. */
  float lastRightFootZRotation; /**< The last z-rotation of the right foot. */

  Vector3BH<> lastFootSpan; /**< The last span between both feet. */
  Pose3DBH lastTorsoMatrix; /**< The last torso matrix for calculating the odometry offset. */

public:
  /** Updates the TorsoMatrixBH representation.
  * @param torsoMatrix The inertia matrix representation which is updated by this module.
  */
  void update(TorsoMatrixBH& torsoMatrix,
          const FilteredSensorData& theFilteredSensorData,
          const RobotDimensionsBH& theRobotDimensions,
          const RobotModel& theRobotModel,
          const GroundContactState& theGroundContactState,
          const DamageConfiguration& theDamageConfiguration);

  /** Updates the OdometryData representation.
  * @param odometryData The odometry data representation which is updated by this module.
  */
  void update(OdometryData& odometryData, const TorsoMatrixBH& theTorsoMatrix);
};
