#include "BeaconDetector.h"
#include <iostream>
#include <cmath>

#define XTHRESHOLD 5
#define YTHRESHOLD 5
#define XDIST 2
#define YDIST 2
#define TBDIVIDE 8

using namespace Eigen;

#define getball() (&vblocks_.world_object->objects_[WO_BALL])
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])
#define getframe() vblocks_.frame_info->frame_id

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) : DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
  candidateCount = 0;
}

void BeaconDetector::detectBeacon(bool topCamera)
{
    // The horizontalBlob elements do not work for the YELLOW and BLUE colors
    blob_detector_->formBlobs(c_PINK);
    blob_detector_->formBlobs(c_YELLOW); // YELLOW -> ORANGE
    blob_detector_->formBlobs(c_BLUE); // BLUE -> FIELD_GREEN
    BlobCollection pinkBlobs = blob_detector_->horizontalBlob[c_PINK];
    BlobCollection yellowBlobs = blob_detector_->horizontalBlob[c_ORANGE];
    BlobCollection blueBlobs = blob_detector_->horizontalBlob[c_FIELD_GREEN];

    findBeacon(&vblocks_.world_object->objects_[WO_BEACON_PINK_YELLOW], pinkBlobs, yellowBlobs, topCamera);
    findBeacon(&vblocks_.world_object->objects_[WO_BEACON_YELLOW_PINK], yellowBlobs, pinkBlobs, topCamera);
    findBeacon(&vblocks_.world_object->objects_[WO_BEACON_BLUE_YELLOW], blueBlobs, yellowBlobs, topCamera);
    findBeacon(&vblocks_.world_object->objects_[WO_BEACON_YELLOW_BLUE], yellowBlobs, blueBlobs, topCamera);
    findBeacon(&vblocks_.world_object->objects_[WO_BEACON_PINK_BLUE], pinkBlobs, blueBlobs, topCamera);
    findBeacon(&vblocks_.world_object->objects_[WO_BEACON_BLUE_PINK], blueBlobs, pinkBlobs, topCamera);
}


// None of the two regions must be 'too big' in comparison with the other
// Case where x and/or y coordinate ranges of one section are greater than 3-3.5 times the corresponding ranges of the other bounding box
bool BeaconDetector::isBigRegion(uint16_t lower1, uint16_t upper1, uint16_t lower2, uint16_t upper2) {
    // Eg. lower right x - upper left x
    return ((lower1 - upper1 + 1) >= 3*(lower2 - upper2 + 1)) || ((lower2 - upper2 + 1) >= 3*(lower1 - upper1 + 1));
}

// The x coordinate of the centroid of each section must lie within the x range values of that of the other section
bool BeaconDetector::isValidCentroid(uint16_t centroidX, uint16_t xi, uint16_t xf) {
    return (centroidX >= xi) && (centroidX <= xf);
}

bool BeaconDetector::findBeacon(WorldObject* beacon, BlobCollection& t, BlobCollection& b, bool topCamera)
{
    if(t.size() > 0 && b.size() > 0) 
    {   cout << "*** finding beacon" << endl;
        for(size_t tn = 0; tn < t.size(); ++tn)
        {
            for(size_t bn = 0; bn < b.size(); ++bn)
            {
                Blob topBlob = t[tn];
                Blob bottomBlob = b[bn];

                cout << "*** top-x " << t[tn].xi << " top-y " << t[tn].yi << endl; 
                cout << "*** bottom-x " << b[bn].xi << " bottom-y " << b[bn].yi << endl; 
                float aspect_ratio = ((float) t[tn].dy) / ((float) t[tn].dx);
                if(abs(aspect_ratio - 1) > 0.20f)
                {
                    cout << "*** aspect ratio " << abs(aspect_ratio - 1) << endl;
                    continue;
                }
                uint16_t diffxpos = abs(t[tn].xi - b[bn].xi);
                if(diffxpos > 0.20 * t[tn].dx)
                {
                    cout << "*** top bottom x offset " << diffxpos << " threshold " << (0.20 * t[tn].dx) << endl;
                    continue;   
                }
                if (!isValidCentroid(topBlob.avgX, bottomBlob.xi, bottomBlob.xf) && !isValidCentroid(bottomBlob.avgX, topBlob.xi, topBlob.xf))
                {
                    cout << "*** centroid" << endl;
                    continue;
                }
                if(bottomBlob.yi < topBlob.yi)
                {
                    continue;
                }
                if(bottomBlob.yi - topBlob.yf > TBDIVIDE)
                {
                    cout << "*** blob" << endl;
                    continue;
                }
                cout << "*** valid beacon" << endl;
                beacon->width = max(topBlob.dx, bottomBlob.dx);
                beacon->height = t[tn].dy + b[bn].dy;
	            beacon->imageCenterX = t[tn].xi + (beacon->width / 2);
	            beacon->imageCenterY = t[tn].yi + (beacon->height / 2);               
                beacon->fromTopCamera = topCamera;
                //beacon->seen = true;
                return true;
            }
        }

    }
    return false;
}
