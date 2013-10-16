#include "BeaconDetector.h"
#include <iostream>
#include <cmath>

#define NUM_BEACON_CANDIDATES 40
#define XTHRESHOLD 5
#define YTHRESHOLD 5
#define XDIST 2
#define YDIST 2
#define TBDIVIDE 8
#define ASPECTTHRESHOLD 0.35f
#define LOWERASPECTTHRESHOLD 0.70f
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
        cout << "********** NumBeaconCandidates: " << candidates.size() << endl;
	for(size_t i = 0; i < candidates.size(); ++i)
	{
		BeaconCandidate& s = candidates[i];
		for(size_t j = 0; j < candidates.size(); ++j)
		{
			BeaconCandidate& c = candidates[j];
			if(i != j && isOverlapping(s.centerX, s.yf, c.xi, c.xf, c.yi, c.yf))
			{
				cout << "**** centerX: " << s.centerX << " s.yf " << s.yf << " c.xi " << c.xi << " c.xf " << c.xf << " c.yi " << c.yi << " c.yf " << c.yf << endl; 
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
			cout << "************************* finding beacon ***********************" << endl;
                        cout << "topColor: " << topColor << " bottomColor: " << bottomColor << endl;
			cout << "topCamera: " << topCamera << endl;
			cout << "*** top-x " << t[tn].xi << " top-y " << t[tn].yi << endl; 
			cout << "*** bottom-x " << b[bn].xi << " bottom-y " << b[bn].yi << endl; 
			if(b[bn].yi < t[tn].yi)
			{
				cout << "*** bottom blob on top of top blob" << endl;
				continue;
			}
			if(b[bn].yi - t[tn].yf > TBDIVIDE)
			{
				cout << "*** distance between top and bottom blob: " << b[bn].yi - t[tn].yf << endl;
				continue;
			}
			uint16_t diffxpos = abs(t[tn].xi - b[bn].xi);
			if(diffxpos > XOFFSET * t[tn].dx)
			{
				cout << "*** top bottom x offset " << diffxpos << " threshold " << (XOFFSET * t[tn].dx) << endl;
				continue;
			}
			if (!isValidCentroid(t[tn].avgX, b[bn].xi, b[bn].xf, 0) && !isValidCentroid(b[bn].avgX, t[tn].xi, t[tn].xf, 0))
			{
				cout << "*** centroid" << endl;
				continue;
			}
			float top_aspect_ratio = ((float) t[tn].dy) / ((float) t[tn].dx);
			float bottom_aspect_ratio = ((float) b[bn].dy) / ((float) b[bn].dx);
			if(abs(top_aspect_ratio - 1) > ASPECTTHRESHOLD || abs(bottom_aspect_ratio - 1) > LOWERASPECTTHRESHOLD)
			{
				if(abs(top_aspect_ratio - 1) > LOWERASPECTTHRESHOLD || abs(bottom_aspect_ratio - 1) > ASPECTTHRESHOLD)
				{
					cout << "*** top aspect ratio " << abs(top_aspect_ratio - 1) << endl;
					cout << "*** bottom aspect ratio " << abs(bottom_aspect_ratio - 1) << endl;
					continue;
				}
			}
			cout << "*** valid beacon" << endl;
			BeaconCandidate bCandidate;
			bCandidate.width = max(t[tn].dx, b[bn].dx);
			bCandidate.height = t[tn].dy + b[bn].dy;
			bCandidate.centerX = t[tn].xi + (bCandidate.width / 2);
			bCandidate.centerY = t[tn].yi + (bCandidate.height / 2);
			bCandidate.valid = true;
			bCandidate.fromTopCamera = topCamera;
			bCandidate.topColor = topColor;
			bCandidate.bottomColor = bottomColor;
                        bCandidate.xi = min(t[tn].xi, b[bn].xi);
			bCandidate.xf = max(t[tn].xf, b[bn].xf);
			bCandidate.yi = t[tn].yi;
			bCandidate.yf = b[bn].yf;
			candidates.push_back(bCandidate);
		}
	}
}
