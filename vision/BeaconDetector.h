#ifndef BEACONDETECTOR_H
#define BEACONDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/Classifier.h>
#include <vision/structures/BeaconCandidate.h>

class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void detectBeacon(bool topCamera);
  void init(TextLogger* tl){textlogger = tl;};
  BeaconCandidate candidates[40];
  int candidateCount;
 private:
  bool findBeacon(WorldObject* beacon, BlobCollection& t, BlobCollection& b, bool topCamera, int* position);
  TextLogger* textlogger;
  Classifier* classifier_;
  BlobDetector* blob_detector_;
  bool isBigRegion(uint16_t lower1, uint16_t upper1, uint16_t lower2, uint16_t upper2);
  bool isValidCentroid(uint16_t centroidX, uint16_t xi, uint16_t xf);
};
#endif
