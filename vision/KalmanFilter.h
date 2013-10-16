#include <vision/BlobDetector.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
using namespace Eigen;

class KalmanFilter {
 public:
  double dt;
    Matrix4f a; // state transition matrix
    Vector4f b; // control input matrix - acceleration
    Matrix4f p; // covariance of the state vector estimate
    Matrix4f q; // state process noise covariance
    Matrix4f r; // measurement noise covariance   
    Matrix4f k; // kalman gain   
    Matrix4f h; // observation - measurement matrix 
    Vector4f x; // state vector estimate Option1: (x, y, velocity-x, velocity-y)
    Vector4f z; // observation vector
    Vector4f u;    // control input value
  
    KalmanFilter();
    void kalman(); 
    void predict(float& xc, float&yc, int n);
};
