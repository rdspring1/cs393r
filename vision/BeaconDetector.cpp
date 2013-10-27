#include "BeaconDetector.h"
#include <iostream>
#include <cmath>
#include <memory/TextLogger.h>

#define NUM_BEACON_CANDIDATES 40
#define XTHRESHOLD 5
#define YTHRESHOLD 5
#define XDIST 2
#define YDIST 2
#define TBDIVIDE 8
#define ASPECTTHRESHOLD 0.35f
#define LOWERASPECTTHRESHOLD 2.0f
#define OVERLAPTHRESHOLD -0.25f
#define XOFFSET 0.70f

using namespace Eigen;

#define getball() (&vblocks_.world_object->objects_[WO_BALL])
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])
#define getframe() vblocks_.frame_info->frame_id

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) : DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) 
{
	candidateCount = 0;
}

void BeaconDetector::detectBeacon(bool topCamera)
{
	candidates.clear();
	candidates.reserve(NUM_BEACON_CANDIDATES);

	// The horizontalBlob elements do not work for the YELLOW and BLUE colors
	blob_detector_->formBlobs(c_PINK);
	blob_detector_->formBlobs(c_YELLOW); // YELLOW -> ORANGE
	blob_detector_->formBlobs(c_BLUE); // BLUE -> FIELD_GREEN
	BlobCollection& pinkBlobs = blob_detector_->horizontalBlob[c_PINK];
	BlobCollection& yellowBlobs = blob_detector_->horizontalBlob[c_ORANGE];
	BlobCollection& blueBlobs = blob_detector_->horizontalBlob[c_FIELD_GREEN];

	findCandidates(pinkBlobs, yellowBlobs, topCamera, c_PINK, c_YELLOW);
	findCandidates(pinkBlobs, blueBlobs, topCamera, c_PINK, c_BLUE);
	findCandidates(yellowBlobs, pinkBlobs, topCamera, c_YELLOW, c_PINK);
	findCandidates(yellowBlobs, blueBlobs, topCamera, c_YELLOW, c_BLUE);
	findCandidates(blueBlobs, yellowBlobs, topCamera, c_BLUE, c_YELLOW);
	findCandidates(blueBlobs, pinkBlobs, topCamera, c_BLUE, c_PINK);

	removeNonBeacons();
	classifyBeacons();

}

void BeaconDetector::removeNonBeacons()
{
	//cout << "********** NumBeaconCandidates: " << candidates.size() << endl;
	for(size_t i = 0; i < candidates.size(); ++i)
	{
		BeaconCandidate& s = candidates[i];
		for(size_t j = 0; j < candidates.size(); ++j)
		{
			BeaconCandidate& c = candidates[j];
			if(i != j && isOverlapping(s.centerX, s.yf, c.xi, c.xf, c.yi, c.yf))
			{
				s.valid = false;
				c.valid = false;
				continue;
			}
		}
	}
}

void BeaconDetector::classifyBeacons()
{
	for(size_t i = 0; i < candidates.size(); ++i)
	{
		BeaconCandidate& c = candidates[i];
		if(c.valid)
		{
			WorldObject* wo = NULL;
			if(c.topColor == c_PINK)
			{
				if(c.bottomColor == c_YELLOW)
				{
					wo = &vblocks_.world_object->objects_[WO_BEACON_PINK_YELLOW]; 
				}
				else
				{
					wo = &vblocks_.world_object->objects_[WO_BEACON_PINK_BLUE]; 
				}
			}
			else if(c.topColor == c_YELLOW)
			{
				if(c.bottomColor == c_PINK)
				{
					wo = &vblocks_.world_object->objects_[WO_BEACON_YELLOW_PINK]; 
				}
				else
				{
					wo = &vblocks_.world_object->objects_[WO_BEACON_YELLOW_BLUE]; 
				}
			}
			else
			{
				if(c.bottomColor == c_YELLOW)
				{
					wo = &vblocks_.world_object->objects_[WO_BEACON_BLUE_YELLOW]; 
				}
				else
				{

					wo = &vblocks_.world_object->objects_[WO_BEACON_BLUE_PINK]; 
				}
			}
			wo->width = c.width;
			wo->height = c.height;
			wo->imageCenterX = c.centerX;
			wo->imageCenterY = c.centerY;
			wo->fromTopCamera = c.fromTopCamera;
			wo->seen = true;
			Position p = cmatrix_.getWorldPosition(wo->imageCenterX, wo->imageCenterY);
			wo->visionDistance = cmatrix_.groundDistance(p);
			float d = cmatrix_.getWorldDistanceByHeight(c.height, 200.0);
			cout << "VISION GROUND DISTANCE " << d << " TOPCOLOR: " << COLOR_NAME(c.topColor) << " BOTTOMCOLOR: " << COLOR_NAME(c.bottomColor) << " CenterX " << wo->imageCenterX << " CenterY " << wo->imageCenterY << endl;
		}
	}
}


// The x coordinate of the centroid of each section must lie within the x range values of that of the other section
bool BeaconDetector::isOverlapping(uint16_t centroidX, uint16_t centroidY, uint16_t xi, uint16_t xf, uint16_t yi, uint16_t yf) 
{
	return (xi <= centroidX) && (centroidX <= xf) && (yi <= centroidY) && (centroidY <= yf);
}

// The x coordinate of the centroid of each section must lie within the x range values of that of the other section
bool BeaconDetector::isValidCentroid(uint16_t centroidX, uint16_t xi, uint16_t xf, uint16_t offset) 
{
	return (xi - offset <= centroidX) && (centroidX <= xf + offset);
}

void BeaconDetector::findCandidates(BlobCollection& t, BlobCollection& b, bool topCamera, Color topColor, Color bottomColor)
{
	for(size_t tn = 0; tn < t.size(); ++tn)
	{
		for(size_t bn = 0; bn < b.size(); ++bn)
		{
			Blob& topBlob = t[tn];
			Blob& bottomBlob = b[bn];
			//cout << "************************* finding beacon ***********************" << endl;
			//cout << "topColor: " << COLOR_NAME(topColor) << " bottomColor: " << COLOR_NAME(bottomColor) << endl;
			//cout << "topCamera: " << topCamera << endl;
			//cout << "*** top-x " << topBlob.xi << " top-y " << topBlob.yi << endl; 
			//cout << "*** bottom-x " << bottomBlob.xi << " bottom-y " << bottomBlob.yi << endl; 
			if(bottomBlob.yi < topBlob.yi && bottomBlob.yf < topBlob.yf)
			{
				//cout << "*** bottom blob above top blob" << endl;
				continue;
			}

			if(bottomBlob.yi - topBlob.yf > TBDIVIDE || bottomBlob.yi - topBlob.yf < OVERLAPTHRESHOLD * topBlob.dy)
			{
				//cout << "*** distance between top and bottom blob: " << bottomBlob.yi - topBlob.yf << endl;
				continue;
			}
			uint16_t diffxpos = abs(topBlob.xi - bottomBlob.xi);
			if(diffxpos > XOFFSET * topBlob.dx)
			{
				//cout << "*** top bottom x offset " << diffxpos << " threshold " << (XOFFSET * topBlob.dx) << endl;
				continue;
			}
			if (!isValidCentroid(topBlob.avgX, bottomBlob.xi, bottomBlob.xf, 0) || !isValidCentroid(bottomBlob.avgX, topBlob.xi, topBlob.xf, 0))
			{
				//cout << "*** centroid" << endl;
				//cout << "*** " << topBlob.avgX << " " << bottomBlob.xi << " " << bottomBlob.xf << endl;
				//cout << "*** " << bottomBlob.avgX << " " << topBlob.xi << " " << topBlob.xf << endl;
				continue;
			}
			if(topBlob.dx > 3 * bottomBlob.dx || bottomBlob.dx > 3 * topBlob.dx || topBlob.dy > 3 * bottomBlob.dy || bottomBlob.dy > 3 * topBlob.dy )
			{
				//cout << "Relative Height and Width" << endl;
				continue;
			}
			BeaconCandidate bCandidate;
			bCandidate.width = max(topBlob.dx, bottomBlob.dx);
			bCandidate.height = topBlob.dy + bottomBlob.dy;

			float top_aspect_ratio = abs(((float) topBlob.dy) / ((float) topBlob.dx) - 1);
			float bottom_aspect_ratio = abs(((float) bottomBlob.dy) / ((float) bottomBlob.dx) - 1);
			if(top_aspect_ratio > ASPECTTHRESHOLD || bottom_aspect_ratio > ASPECTTHRESHOLD)
			{
				float aspect_ratio = bCandidate.height / bCandidate.width;
				if(aspect_ratio > 2 - ASPECTTHRESHOLD && aspect_ratio < 4 + ASPECTTHRESHOLD)
				{
					//cout << "Partial Beacon Detection TOPCOLOR: " << COLOR_NAME(topColor) << " BOTTOMCOLOR: " << COLOR_NAME(bottomColor) << endl;
					visionLog((1, "Partial Beacon Detected TOPCOLOR: %s BOTTOMCOLOR: %s TOPX %d TOPY %d", COLOR_NAME(topColor), COLOR_NAME(bottomColor), topBlob.xi, topBlob.yi));
				}
				//cout << "*** Partial Ratio " << aspect_ratio << endl;
				//cout << "*** top aspect ratio " << top_aspect_ratio << endl;
				//cout << "*** bottom aspect ratio " << bottom_aspect_ratio << endl;
				continue;
			}
			//cout << "*** valid beacon" << endl;
			bCandidate.centerX = topBlob.xi + (bCandidate.width / 2);
			bCandidate.centerY = topBlob.yi + (bCandidate.height / 2);
			bCandidate.valid = true;
			bCandidate.fromTopCamera = topCamera;
			bCandidate.topColor = topColor;
			bCandidate.bottomColor = bottomColor;
			bCandidate.xi = min(topBlob.xi, bottomBlob.xi);
			bCandidate.xf = max(topBlob.xf, bottomBlob.xf);
			bCandidate.yi = topBlob.yi;
			bCandidate.yf = bottomBlob.yf;
			candidates.push_back(bCandidate);
		}
	}
}
