#include "Classifier.h"
#include <iostream>
#include <vector>

using namespace std;
Classifier::Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera) :
    vblocks_(vblocks), vparams_(vparams), iparams_(iparams), camera_(camera), initialized_(false) {
  segImg_ = new unsigned char[iparams.size];
  segImgLocal_ = segImg_;
  setStepScale(iparams_.defaultHorizontalStepScale, iparams_.defaultVerticalStepScale);
  
  horizontalPointCount = new uint32_t*[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    horizontalPointCount[i] = new uint32_t[iparams_.height];
    memset(horizontalPointCount[i], 0, iparams_.height);
  }
  verticalPointCount = new uint32_t*[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    verticalPointCount[i] = new uint32_t[iparams_.width];
    memset(verticalPointCount[i], 0, iparams_.width);
  }

  for(int i = 0; i<NUM_COLORS; i++)
  {
    for(int j = 0; j < iparams_.height; j++)
        horizontalPointCount[i][j] = 0; // issues with memset

    for(int j = 0; j < iparams_.width; j++)
        verticalPointCount[i][j] = 0;
  }
  
  // Initialize horizontalPoint
  horizontalPoint = new VisionPoint**[NUM_COLORS];
  for (int i = 0; i < NUM_COLORS; ++i) {

    horizontalPoint[i] = new VisionPoint*[iparams_.height];

    for (int j = 0; j < iparams_.height; ++j) {
       horizontalPoint[i][j] = new VisionPoint[iparams_.width];
    }
  }
}

Classifier::~Classifier() {
  delete [] segImgLocal_;
}

bool Classifier::setImagePointers() {
  bool imageLoaded = vblocks_.image->loaded_;
  if(vblocks_.image == NULL) {
    printf("No image block loaded! Classification failed.\n");
    return false;
  }
  if(vblocks_.robot_vision == NULL) {
    printf("No vision block loaded! Classification failed.\n");
    return false;
  }
  if(camera_ == Camera::TOP) {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgTop(segImg_);
      img_ = vblocks_.image->getImgTop();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->loaded_) {
      segImg_ = vblocks_.robot_vision->getSegImgTop();
    }
    #endif
  }
  else {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgBottom(segImg_);
      img_ = vblocks_.image->getImgBottom();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->loaded_) {
      segImg_ = vblocks_.robot_vision->getSegImgBottom();
    }
    #endif
  }
  if(!initialized_) {
    #ifdef TOOL
    if(imageLoaded)
    #endif
    memset(segImg_, c_UNDEFINED, sizeof(unsigned char) * iparams_.size);
    initialized_ = true;
  }
  return true;
}

bool Classifier::classifyImage(unsigned char *colorTable) {
  if(!setImagePointers()) return false;
  FocusArea area(0, 0, iparams_.width - 1, iparams_.height - 1);
  classifyImage(area, colorTable);
  return true;
}

void Classifier::classifyImage(const std::vector<FocusArea>& areas, unsigned char *colorTable) {
  if(!setImagePointers()) return;
  for(unsigned int i = 0; i < areas.size(); i++)
    classifyImage(areas[i], colorTable);
}


void Classifier::classifyImage(const FocusArea& area, unsigned char* colorTable){
  bool imageLoaded = vblocks_.image->loaded_;
  if(!imageLoaded) {
    visionLog((20, "Classifying with no raw image"));
  }
  colorTable_ = colorTable;
  visionLog((28, "Classifying on area %i,%i to %i,%i with horizon %2.f,%2.f", area.x1, area.y1, area.x2, area.y2, horizon_.gradient, horizon_.offset));
  for (int y = area.y1; y <= area.y2; y += vstep_) {
    for(int x = area.x1; x <= area.x2; x += hstep_) {
      Color c;
#ifdef TOOL
      if (imageLoaded) // if a raw image is available
#endif
      {
        c = ColorTableMethods::xy2color(img_, colorTable, x, y, iparams_.width);
        if(horizon_.exists && c == c_ORANGE && !horizon_.isAbovePoint(x, y)) continue;
        if(horizon_.exists && c == c_WHITE && !horizon_.isAbovePoint(x,y)) c = c_ROBOT_WHITE; // We shouldn't be handling lines above the horizon
        segImg_[iparams_.width * y + x] = c;
      }
    }
  }
}

void Classifier::setStepScale(int h, int v){
    hstep_ = (1 << h);
    vstep_ = (1 << v);
    hscale_ = h;
    vscale_ = v;
}

void Classifier::getStepSize(int& h, int& v){
    h = hstep_;
    v = vstep_;
}

void Classifier::getStepScale(int& h, int& v){
    h = hscale_;
    v = vscale_;
}

void Classifier::reset(){
    // Reset matrices for when constructing a new run
    for(int color = 0; color < NUM_COLORS; color++)
    {
      for(int j = 0; j < iparams_.height; j++)
         horizontalPointCount[color][j] = 0; // issues with memset

      for(int j = 0; j < iparams_.width; j++)
        verticalPointCount[color][j] = 0;

        for(int j = 0; j < iparams_.height; j++) {
            for( int k = 0; k < iparams_.width; k++){
                horizontalPoint[color][j][k].children.clear();
                horizontalPoint[color][j][k].xf = 0;
                horizontalPoint[color][j][k].xi = 0;
                horizontalPoint[color][j][k].yf = 0;
                horizontalPoint[color][j][k].yi = 0;
                horizontalPoint[color][j][k].parent = &horizontalPoint[color][j][k];
                horizontalPoint[color][j][k].position = 0;
                horizontalPoint[color][j][k].number = 0;
            }
        }
     }  
}

void Classifier::constructRuns(){
  reset();

  int previousColor;
  int xi;

  // Horizontal runs
  for(int y = 0; y < iparams_.height; ++y) // row
  {
    xi = 0;
    previousColor = getSegPixelValueAt(xi, y); // The first color seen for the row is the one at position x, 0

    horizontalPointCount[previousColor][y]++; // Increase the count for the run for that color in that y
    
    for(int x = 1; x < iparams_.width; ++x) // we proceed at x = 1
    {
      int currentColor = getSegPixelValueAt(x, y);

      // If the current color is the same as the previous, we are in the same run
      if (currentColor != previousColor){
        // Get the number of run for the previous color
        int runNumber = horizontalPointCount[previousColor][y] - 1; // the run number is the count of runs - 1
        
        // Store previous run in horizontalPoint
        VisionPoint* vp = &horizontalPoint[previousColor][y][runNumber];
        vp->xi = xi;
        vp->xf = x - 1; // previous x
        vp->yi = y;
        vp->yf = y;
        
        vp->position = y;
        vp->number = runNumber;
        
        vp->parent = vp; // Initially a run establishes itself as its parent
        vp->children.push_back(vp); // A runs adds itself as a child of its parent (itself)
        
        // Store the new initial xi
        xi = x;
        previousColor = currentColor;
        horizontalPointCount[currentColor][y]++; // Increment number of runs for that color in that row (origin)
      } // if
    } // for x
    // store the last run for that row

    int runNumber = horizontalPointCount[previousColor][y] - 1;
    VisionPoint* vp = &horizontalPoint[previousColor][y][runNumber];
    vp->xi = xi;
    vp->xf = iparams_.width - 1;
    vp->yi = y;
    vp->yf = y;
    vp->number = runNumber;
    vp->parent = vp;

  } // for y
}

// horizontal runs
// returns the root of the given run for the given color
VisionPoint * Classifier::root(VisionPoint *run) {  
 VisionPoint *root = run;
  
  // Get the root of the run
 //while (((root->parent)->position != root->position) && ((root->parent)->number != root->number)) {
   while ((root->parent) != root) {
    root = root->parent;
  }
  
  // Path compression
  while (run != root) {
    VisionPoint *newRun = run->parent;
    changeChildren(run, root);
    (run->parent) = root;
    run = newRun;
  }

   return root;  
}

// Checks the runs of each color, if there's a run above that it can be merged with it calls unionOfRuns()
void Classifier::mergeRuns() {

  //cout << " Start merging runs " << endl;
  for (int color = 0; color < NUM_COLORS; ++color) {

    if(color == c_WHITE || color == c_UNDEFINED || color == c_ORANGE || color == c_ROBOT_WHITE || color == c_FIELD_GREEN)
        continue;

    for (int y = 1; y < iparams_.height; ++y) {
      // Get number of runs for that color at that row
      int numRuns = horizontalPointCount[color][y];
      int numRunsAbove = horizontalPointCount[color][y - 1]; // number of runs of that color in the row above
      
      // Check if there's a run above that we can merge it with
      if (numRuns > 0 && numRunsAbove > 0) {
        for (int runNum = 0; runNum < numRuns; ++runNum) {
          VisionPoint* run = &horizontalPoint[color][y][runNum];
          vector<VisionPoint *> runsAbove; // stores all the potential runs that are above that can be a parent
          
          // Check a potential run above to merge it with
          for (int runAboveNum = 0; runAboveNum < numRunsAbove; ++runAboveNum) {
              VisionPoint* runAbove = &horizontalPoint[color][y - 1][runAboveNum];
              
              if ( ((run->xi >= runAbove->xi) && (run->xi <= runAbove->xf))
                   || ((run->xf >= runAbove->xi) && (run->xf <= runAbove->xf))) {
                
                // put the root of all valid runs above in a vector
                runsAbove.push_back(root(runAbove));         
              }
            } // for runAbove
           if (runsAbove.size() > 0) {
              // union with the top most, left most root
              int runY = iparams_.height;
              int runXi = iparams_.width;
              VisionPoint *root1 = NULL;
              for (size_t i = 0; i < runsAbove.size(); ++i) {
                  VisionPoint *r = root(runsAbove[i]);
                    
                   if ((r->position < runY) || ((r->position == runY) && (r->xi < runXi))){ // if that root is higher, that's the new root
                      runY = r->position;
                      runXi = r->xi;
                      root1 = r;
                    }
               }
              // update all the other roots to point to the root that we found
              if (root1 != NULL) {
                  if (color == 3){
                    //cout << "** Updating runsAbove to point to the new root" << endl;
                   }
                 update(root1, runsAbove);
                 run->parent = root1; // make the run point to the root
                 changeChildren(run, root1); // update the children from the run
                }
              runsAbove.clear();
            }
        } // for run
      } //if
    } // for y
  } 
  //cout << " Finished merging runs " << endl;
}

// Makes all the runs and their children point to the given root
void Classifier::update(VisionPoint *root, vector<VisionPoint *> runs) {
    for (size_t i = 0; i < runs.size(); ++i) {
        if (runs[i] == root)
            continue; // if the run is the root, don't do anything
        runs[i]->parent = root; // Change the parent of the run to be the root.
        changeChildren(runs[i], root);
    }
}

// Makes all children of run point to the root
void Classifier::changeChildren(VisionPoint *run, VisionPoint *root) {
    
    for (size_t i = 0; i < run->children.size(); ++i) {
       (run->children[i])->parent = root;
       (root->children).push_back(run->children[i]); // Add the children to the root
    }
    run->children.clear(); // Delete children from run
}

// Union of two runs
void Classifier::unionOfRuns(VisionPoint *run1, VisionPoint *run2) {
  VisionPoint *root1 = root(run1);
  VisionPoint *root2 = root(run2);
  
  // compare the size of the children array not the variable size
  if (root1->children.size() < root2->children.size()) {
      changeChildren(root1, root2);
      root1->parent = root2;
    }
  else {
    changeChildren(root2, root1);
    root2->parent = root1;
  }
}




