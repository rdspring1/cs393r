#include <vision/BlobDetector.h>

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) : 
  DETECTOR_INITIALIZE, classifier_(classifier) {
    horizontalBlob.resize(NUM_COLORS);
    verticalBlob.resize(NUM_COLORS);
}
<<<<<<< HEAD

// vector<BlobCollection> horizontalBlob
void BlobDetector::formBlobs(Color blobColor) {
	VisionPoint ***hp = classifier_->horizontalPoint;

	//cout << "Forming blobs..." << endl;
	int hbpos = blobColor;

	if(blobColor == c_BLUE)
		hbpos = c_FIELD_GREEN;

	if(blobColor == c_YELLOW)
		hbpos = c_ORANGE;

	horizontalBlob[hbpos].clear();
	int numBlobs = 0;
	int numBlobs2 = 0;
	for (int y = 0; y < iparams_.height; y+=2) {
		int numRuns = classifier_->horizontalPointCount[blobColor][y];

		for (int i = 0; i < numRuns; ++i) {
			vector<VisionPoint*> children = hp[blobColor][y][i].children;

			if (children.size() > 2) {
				++numBlobs;
				// get bounding box
				uint16_t xi, xf, dx, yi, yf, dy, sumY, sumX = 0;

				xi = iparams_.width;
				xf = 0;
				yi = iparams_.height;
				yf = 0;
				sumY = sumX = 0;
				int numCorrectColorPixels = 0;
				for(size_t c = 0; c < children.size(); ++c) {
					if (children[c]->xi < xi)
						xi = children[c]->xi;
					if (children[c]->yi < yi)
						yi = children[c]->yi;
					if (children[c]->xf > xf)
						xf = children[c]->xf;
					if (children[c]->yf > yf)
						yf = children[c]->yf;
					sumY += yi;
					sumX += xi + dx/2;
				}
				dx = xf - xi + 1;
				dy = yf - yi + 1;

				for(int a = xi; a <= xf; ++a) 
				{
					for(int b=yi; b<=yf; ++b) 
					{
						if(getSegPixelValueAt(a, b) == blobColor)
							numCorrectColorPixels++;
					}
				}
				float area = abs(dx) * 1.0 * abs(dy);
				float ratio = numCorrectColorPixels / area;
				if (ratio > 0.65  && area > 225) {
					numBlobs2++;
					Blob b;
					b.xi = xi;
					b.xf = xf;
					b.yi = yi;
					b.yf = yf;
					b.dx = dx;
					b.dy = dy;
					b.correctPixelRatio = ratio;
					b.numCorrectColorPixels = numCorrectColorPixels;
					b.avgX = xi + dx / 2;
					b.avgY = yi + dy / 2;
					assert(b.xi + b.dx - 1< iparams_.width);
					assert(b.xf < iparams_.width);
					assert(b.yi + b.dy - 1 < iparams_.height);
					assert(b.yf < iparams_.height);
					assert(b.dx < iparams_.width);
					assert(b.dy < iparams_.height);
					assert(b.dx > 0);
					assert(b.dy > 0);
					assert(b.correctPixelRatio > 0);
					assert(b.numCorrectColorPixels > 0);

					horizontalBlob[hbpos].push_back(b);
				}
			}
		}
		BlobCollection mergedBlobs = mergeBlobs(horizontalBlob[hbpos], XTHRESHOLD, YTHRESHOLD);
		horizontalBlob[hbpos].clear();
		for(size_t i = 0; i < mergedBlobs.size(); ++i)
		{
			horizontalBlob[hbpos].push_back(mergedBlobs[i]);
		}
		//cout << "Blobs color " << blobColor << " : " << numBlobs << " numBlobs2 " << numBlobs2 << " real size " << horizontalBlob[blobColor].size() << endl;
	}
	//cout << "End forming blobs..." << endl;
}

=======
>>>>>>> 2a3dcdf948dddfc5f02e3e8efb07c4ef1382bff6
