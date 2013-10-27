#include <localization/LocalizationModule.h>
#include <cmath>

void LocalizationModule::specifyMemoryDependency() {
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

void LocalizationModule::specifyMemoryBlocks() {
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


void LocalizationModule::initSpecificModule(){
  for(int i = 0; i < NUM_PARTICLES; i++) {
    Particle& p = particles_[i];
    p.loc.x = 0;
    p.loc.y = 0;
    p.theta = 0;
    p.prob = 1.0;
  }
  resetParticles();
  copyParticles();
}

void LocalizationModule::processFrame() {
  int frameID = frameInfo->frame_id;
  // 1. Update particles from observations
    updateParticlesFromObservations();

  // 2. If this is a resampling frame, resample
  //resample();
  // 3. Update the robot's pose
  // 4. If this is a random walk frame, random walk
  // 5. Copy particles to localization memory:
  copyParticles();  
}

float LocalizationModule::getTotalFitness() {
    float total = 0.0f;
    for(int i = 0; i < NUM_PARTICLES; i++) {
        total += particles_[i].prob;
    }

    return total;
}

void LocalizationModule::resample() {
   float totalFitness = getTotalFitness();
   float c[NUM_PARTICLES];
   c[0] = particles_[0].prob;

   float p = totalFitness *1.0 / NUM_PARTICLES;
   Particle particlescpy[NUM_PARTICLES] = particles_;
   float start = (float)rand()/((float)RAND_MAX/(p)); // indicates where the first pointer will start

   for (int i = 1; i < NUM_PARTICLES; ++i) {
      c[i] = c[i-1] + particles_[i].prob;
    }
  int i = 0;
  float u = start;
  for (int j = 0; j < NUM_PARTICLES; ++j) {
     while (u > c[i])
        i++;
    // add particle
    particles_[j] = particlescpy[i];
    u +=  p;
  } 
}

vector<WorldObject *> LocalizationModule::getBeacons() {
    vector<WorldObject *> beacons;

    WorldObject& wo = worldObjects->objects_[WO_BEACON_PINK_YELLOW];
   /* if (wo.seen)
       beacons.push_back(&wo);
    wo =worldObjects->objects_[WO_BEACON_PINK_BLUE];
     if (wo.seen)
       beacons.push_back(&wo);
    wo = worldObjects->objects_[WO_BEACON_YELLOW_PINK];
     if (wo.seen)
       beacons.push_back(&wo);*/
    wo = worldObjects->objects_[WO_BEACON_YELLOW_BLUE]; 
    wo.visionDistance = 1500/2;
     //if (wo.seen)
       beacons.push_back(&wo);
   /*wo = worldObjects->objects_[WO_BEACON_BLUE_YELLOW];
     if (wo.seen)
       beacons.push_back(&wo);
    wo = worldObjects->objects_[WO_BEACON_BLUE_PINK]; 
*/
    return beacons;
}


void LocalizationModule::updateParticlesFromObservations() {
    vector<WorldObject*> beacons = getBeacons();
    float totalweight = 0.0;
    bool valueGreaterThan1 = false;
    for(int i = 0; i < NUM_PARTICLES; i++) {
        Particle& p = particles_[i];

        // compute the distance to each seen beacon
        for(int j = 0; j < beacons.size(); ++j) {
           float distance = p.loc.getDistanceTo(beacons[j]->loc);
           float observed = beacons[j]->visionDistance;
           float diff = fabs(observed - distance);
           p.prob += observed / (observed + diff); 
           if (p.prob > 1) 
                valueGreaterThan1 = true;
           totalweight += p.prob;
        }
  }

  // only normalize if we got a value greater than 1
  // normalize because we added probabilities of distances of 2 or more beacons
  if (valueGreaterThan1) 
  {
      for(int i = 0; i < NUM_PARTICLES; ++i) 
      {
        particles_[i].prob /= totalweight;
      }
  }
}

void LocalizationModule::copyParticles() {
  memcpy(localizationMem->particles, particles_, NUM_PARTICLES * sizeof(Particle));
}

void LocalizationModule::updateParticlesFromOdometry() {
  Pose2D disp = odometry->displacement;
  for(int i = 0; i < NUM_PARTICLES; i++) {
    Particle& p = particles_[i];
    p.moveRelative(disp);
    p.degradeProbability(DEGRADE_FACTOR);
  }
}

void LocalizationModule::resetParticles(){
  for (int i = 0; i < NUM_PARTICLES; i++){
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

    float p1AngleRatio = p1Ratio;
    float p2AngleRatio = p2Ratio;

    part1.move(dPos*p1Ratio, p1AngleRatio*dAng);
    part2.move(-dPos*p2Ratio,p2AngleRatio*-dAng);

  }
}

void LocalizationModule::updatePose() {
  WorldObject& self = worldObjects->objects_[robotState->WO_SELF];
  // Compute a weighted average of the particles to fill in your location
}
