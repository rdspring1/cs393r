#ifndef BEACONDETECTOR_H
#define BEACONDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/Classifier.h>
#include <vision/enums/Colors.h>
#include <vision/structures/BeaconCandidate.h>

class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector);
  void detectBeacon(bool topCamera);
  void init(TextLogger* tl){textlogger = tl;};
  std::vector<BeaconCandidate> candidates;
  int candidateCount;
 private:
  void findBeacon(BlobCollection& t, BlobCollection& b, bool topCamera, Color topColor, Color BottomColor)
  void classifyBeacons();
  void removeNonBeacons();
  TextLogger* textlogger;
  Classifier* classifier_;
  BlobDetector* blob_detector_;
  bool isValidCentroid(uint16_t centroidX, uint16_t xi, uint16_t xf, uint16_t offset);
  bool isOverlapping(uint16_t centroidX, uint16_t centroidY, uint16_t xi, uint16_t xf, uint16_t yi, uint16_t yf) 
};
#endif
