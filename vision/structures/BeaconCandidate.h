#ifndef BEACON_CANDIDATE_H
#define BEACON_CANDIDATE_H

#include <vision/structures/Position.h>
#include <vision/structures/Blob.h>
#include <vision/structures/BeaconCandidate.h>

struct BeaconCandidate {
  unsigned int index;
  float centerX;
  float centerY;
  float radius;
  float stddev;
  float width;
  float height;
  float groundDistance;
  float confidence;
  float kwDistanceDiscrepancy;
  Blob* blob;
  Position relPosition;
  Position absPosition;
  bool valid;
  bool fromTopCamera;
  Color topColor;
  Color bottomColor;
  BeaconCandidate() : blob(NULL), valid(false) { }
};

#endif
