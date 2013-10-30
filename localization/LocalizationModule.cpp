#include <localization/LocalizationModule.h>
#include <cmath>
#include <iostream>
#include <algorithm>
using namespace std;

#define RAND_THRESHOLD 40
#define RESAMPLE_RATE 2
#define RWALK_RATE 3
#define GOOD_DISTANCE_DIFF_THRESH 0.25f
#define MED_DISTANCE_DIFF_THRESH 0.50f
#define VISION_BEARING 75.0f
#define MIN_RAND_WALK 0.10f
#define NEW_RAND_WALK 0.60f
#define RESET_FRAME 1000

void LocalizationModule::specifyMemoryDependency() 
{
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("team_packets");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("behavior");
  requiresMemoryBlock("vision_processed_sonar");
  requiresMemoryBlock("delayed_localization");
}

void LocalizationModule::specifyMemoryBlocks() 
{
  getOrAddMemoryBlock(worldObjects,"world_objects");
  getOrAddMemoryBlock(localizationMem,"localization");
  getOrAddMemoryBlock(teamPacketsMem,"team_packets");
  getOrAddMemoryBlock(frameInfo,"vision_frame_info");
  getOrAddMemoryBlock(odometry,"vision_odometry");
  getOrAddMemoryBlock(robotState,"robot_state");
  getOrAddMemoryBlock(gameState,"game_state");
  getOrAddMemoryBlock(jointAngles,"vision_joint_angles");
  getOrAddMemoryBlock(behaviorMem,"behavior");
  getOrAddMemoryBlock(processedSonar,"vision_processed_sonar");
  getOrAddMemoryBlock(delayedLocalization,"delayed_localization");
}


void LocalizationModule::initSpecificModule()
{
  for(int i = 0; i < NUM_PARTICLES; i++) 
  {
    Particle& p = particles_[i];
    p.loc.x = 0;
    p.loc.y = 0;
    p.theta = 0;
    p.prob = 1.0;
  }
  resetParticles();
  copyParticles();
}

void LocalizationModule::processFrame() 
{
    int frameID = frameInfo->frame_id;

    // Fail-Safe
    if (frameID % RESET_FRAME == 0) 
    {
        resetParticles();
    }

    // Update particles from observations
    updateParticlesFromOdometry();
    updateParticlesFromObservations();

    // If this is a resampling frame, resample
    // resample every 5 frames
    if (frameID % RESAMPLE_RATE == 0) 
    {
        resample();
    }

    // Update the robot's pose
    updatePose();

    // If this is a random walk frame, random walk
    if (frameID % RWALK_RATE == 0) 
    {
        randomWalkParticles();
    }

    // Copy particles to localization memory:
    copyParticles();  
}

// Calculates the total weight of all the particles
float LocalizationModule::getTotalFitness() 
{
    float total = 0.0f;
    for(int i = 0; i < NUM_PARTICLES; i++) 
    {
        total += particles_[i].prob;
    }
    return total;
}

void LocalizationModule::resample() 
{
    float totalFitness = getTotalFitness();
    float c[NUM_PARTICLES];

    float p = totalFitness * 1.0f / NUM_PARTICLES;
    Particle particlescpy[NUM_PARTICLES] = particles_;
    int randnum = rand() % RAND_THRESHOLD;
    float randfraction = randnum * 1.0f / RAND_THRESHOLD;
    float start = randfraction * p; // indicates where the first pointer will start

    c[0] = particles_[0].prob;
    for (int i = 1; i < NUM_PARTICLES; ++i) 
    {
        c[i] = c[i-1] + particles_[i].prob;
    }

    int i = 0;
    float u = start;
    for (int j = 0; j < NUM_PARTICLES; ++j) {
         while (u > c[i])
         {
            i++;
         }
        // add particle
        particles_[j] = particlescpy[i];
        u += p;
    } 
}

vector<WorldObject *> LocalizationModule::getBeacons() {
    vector<WorldObject*> beacons;
    if (worldObjects->objects_[WO_BEACON_PINK_YELLOW].seen)
    {
       beacons.push_back(&worldObjects->objects_[WO_BEACON_PINK_YELLOW]);
    }
    if (worldObjects->objects_[WO_BEACON_PINK_BLUE].seen)
    {
       beacons.push_back(&worldObjects->objects_[WO_BEACON_PINK_BLUE]);
    }
    if (worldObjects->objects_[WO_BEACON_YELLOW_PINK].seen)
    {
       beacons.push_back(&worldObjects->objects_[WO_BEACON_YELLOW_PINK]);
    }
    if (worldObjects->objects_[WO_BEACON_YELLOW_BLUE].seen)
    {
       beacons.push_back(&worldObjects->objects_[WO_BEACON_YELLOW_BLUE]);
    }
    if (worldObjects->objects_[WO_BEACON_BLUE_YELLOW].seen)
    {
       beacons.push_back(&worldObjects->objects_[WO_BEACON_BLUE_YELLOW]);
    }
    if (worldObjects->objects_[WO_BEACON_BLUE_PINK].seen)
    {
       beacons.push_back(&worldObjects->objects_[WO_BEACON_BLUE_PINK]);
    }
    return beacons;
}


float LocalizationModule::maxProb() 
{
    float max = 0.0f;
    for (int i = 0; i < NUM_PARTICLES; ++i) 
    {
        max = std::max(max, particles_[i].prob);
    }
    return max;
}

void LocalizationModule::normalize() {
    float max = maxProb();
    for (int i = 0; i < NUM_PARTICLES; ++i) 
    {
        particles_[i].prob /= max;
       // cout << "*** prob " << particles_[i].prob << " x " << particles_[i].loc.x << " y " << particles_[i].loc.y << endl;
    }
}

float LocalizationModule::variance() {
    float mean = getTotalFitness() / NUM_PARTICLES;
    float variance = 0.0f;
    for (int i = 0; i < NUM_PARTICLES; ++i) {
       variance = pow((particles_[i].prob - mean), 2) / NUM_PARTICLES;
    }
    return variance;
}

void LocalizationModule::updateParticlesFromObservations() {
    vector<WorldObject*> beacons = getBeacons();
    float countFarAvg = 0.0f;
    //cout << "*** NUM beacons " << beacons.size() << endl;
    for(int i = 0; i < NUM_PARTICLES; i++) {
        Particle& p = particles_[i];
 
        // compute the distance to each seen beacon
        for(size_t j = 0; j < beacons.size(); ++j) {
           float distance = p.loc.getDistanceTo(beacons[j]->loc);
           float observed = beacons[j]->visionDistance;
           float diff = fabs(observed - distance); // difference between the observed distance and the distance from the particle

           // check the angle of the particle to the beacon
           Pose2D bPose(0, beacons[j]->loc.x, beacons[j]->loc.y);
           Pose2D pPose(p.theta, p.loc.x, p.loc.y);
           Pose2D bPoseRel = bPose.globalToRelative(pPose);
           AngRad relBearing = atan2f(bPoseRel.translation.y, bPoseRel.translation.x);

           if (diff / distance < GOOD_DISTANCE_DIFF_THRESH) 
           {
                // if the relBearing is greater than 150, the beacon is not in our range of vision
                if (fabs(relBearing) > (VISION_BEARING * M_PI / 180.0)) {
                  p.degradeProbability(0.75);
                }
                else 
                {
                  p.degradeProbability(1.0);
                }
           }
           else 
           {
                countFarAvg++;
                p.degradeProbability(0.5);
           }
        }
    }
    //averageDistanceDiff = fabs(observed - averageDistance);
    countFarAvg /= (NUM_PARTICLES * beacons.size());
    if(countFarAvg > 0.7)
        areFar = true;
    else
        areFar = false;
    normalize(); //normalize so that we always have a value with 1.0 probability
}

void LocalizationModule::copyParticles() 
{
  memcpy(localizationMem->particles, particles_, NUM_PARTICLES * sizeof(Particle));
}

void LocalizationModule::updateParticlesFromOdometry() 
{
  Pose2D disp = odometry->displacement;
  for(int i = 0; i < NUM_PARTICLES; i++) {
    Particle& p = particles_[i];
    p.moveRelative(disp);
    p.degradeProbability(DEGRADE_FACTOR);
  }
}

void LocalizationModule::resetParticles()
{
  for (int i = 0; i < NUM_PARTICLES; i++)
  {
    Particle& p = particles_[i];
    p.prob = 1.0f;
    p.placeRandomly();
  }
}

void LocalizationModule::setParticleProbabilities(float newProb){
  for (int i = 0; i < NUM_PARTICLES; i++){
    Particle& p = particles_[i];
    p.prob = newProb;
  }
}

void LocalizationModule::randomWalkParticles() {
  // loop through half, moving each pair of particles opposite directions

  for ( int i = 0; i < NUM_PARTICLES/2; i++ ) {
    Particle& part1 = particles_[i];
    Particle& part2 = particles_[i+NUM_PARTICLES/2];

    Vector2D dPos(DELTA_DIST * (2.0 * drand48() - 1),
                  DELTA_DIST * (2.0 * drand48() - 1));
    AngRad dAng = DELTA_ANG  * (2.0 * drand48() - 1);

    // move them in opposite directions on this vector, based on their prob
    float p1Ratio = 1.0 - part1.prob;
    float p2Ratio = 1.0 - part2.prob;
    p1Ratio = (p1Ratio < MIN_RAND_WALK) ? NEW_RAND_WALK : p1Ratio; 
    p2Ratio = (p2Ratio < MIN_RAND_WALK) ? NEW_RAND_WALK : p2Ratio;
        
    float p1AngleRatio = 1.0 - part1.prob;
    float p2AngleRatio = 1.0 - part2.prob;
    p1Ratio = (p1Ratio < MIN_RAND_WALK) ? MIN_RAND_WALK : p1Ratio; 
    p2Ratio = (p2Ratio < MIN_RAND_WALK) ? MIN_RAND_WALK : p2Ratio;

    part1.move(dPos*p1Ratio, p1AngleRatio*dAng);
    part2.move(-dPos*p2Ratio,p2AngleRatio*-dAng);
    }
}

void LocalizationModule::updatePose() {
    WorldObject& self = worldObjects->objects_[robotState->WO_SELF];
    // Compute a weighted average of the particles to fill in your location
    float x = 0.0f;
    float y = 0.0f;
    float bearing = 0.0f;
    for (int i = 0; i < NUM_PARTICLES; ++i) 
    {
        float weight = particles_[i].prob;
        x += weight * particles_[i].loc.x;
        y += weight * particles_[i].loc.y;
        bearing += weight * particles_[i].theta;
    }

   Point2D origin;
   float total = getTotalFitness();
   self.loc.x = x / total;
   self.loc.y = y / total;
   self.orientation = bearing / total;
   self.visionConfidence = variance();
   self.visionDistance = self.loc.getDistanceTo(origin);
   self.visionBearing = self.loc.getBearingTo(origin, self.orientation);
}
