#include "BlobDetector.h"
#include "KalmanFilter.h"
#include <cmath>
#include <iostream>

using namespace std;

using namespace Eigen;

    
KalmanFilter::KalmanFilter()
{
    dt = 1.0f;
  
    a <<
    1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0, 
    0, 0, 0, 1;
    
    b[0] = dt*dt/2;
    b[1] = dt*dt/2;
    b[2] = dt;
    b[3] = dt;
    
    // acceleration value
	//u = -1; 

    // system process noise covariance in milimeters
    float acceleration_sd = 1; 
    float dt4 = pow(dt,4)/4;
    float dt3 = pow(dt,3)/2;
    float dt2 = pow(dt,2);
    q <<
        dt4, 0, dt3, 0,
        0, dt4, 0, dt3,
        dt3, 0, dt2, 0,
        0, dt3, 0, dt2;
    q = q * pow(acceleration_sd, 2);

    h << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    
    // measurement covariance milimeters
    float measurement_variance = 1.4f;
    r <<
	    1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    r = r * pow(measurement_variance, 2);

	x = Vector4f::Zero();
	z = Vector4f::Zero();
    u = Vector4f::Zero();
  }

// Run predict n times
void KalmanFilter::predict(float& xc, float&yc, int n)
{
  Vector4f xcopy = x;
  Matrix4f pcopy = p;

    for(int i=1; i<=n; ++i)
    {
        xcopy = a*x + b.cwiseProduct(u);
        pcopy = a*p*a.transpose() + q;
    }
    xc = xcopy[0];
    yc = xcopy[1];
    //printf("%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n", xcopy[0], xcopy[1], xcopy[2], xcopy[3], z[0], z[1], z[2], z[3]);
}

void KalmanFilter::kalman()
{  
  // If the initial state is unavailable, initialize using observation values:
  if (x == Vector4f::Zero())
  {
    x = z;
    p = q;
  }
  else
  {
    //calculate observed velocities
    z[2] = z[0] - x[0];
    z[3] = z[1] - x[1];

    // Update Acceleration Control
    u[0] = -0.20 * x[2];
    u[1] = -0.25 * x[3];
    u[2] = -0.20 * x[2];
    u[3] = -0.25 * x[3];
    //cout << "** u: " << endl << u << endl;

    // Prediction step
    x = a * x + b.cwiseProduct(u);
    p = a * p * a.transpose() + q;

    //printf("** Prior: %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n", x[0], x[1], x[2], x[3], z[0], z[1], z[2], z[3]);

    // Kalman gain
    k = p * h.transpose() * (h * p * h.transpose() + r).inverse();
    //cout << "** k: " << endl << k << endl;

    // Correction step
    x = x + k * (z - h * x);
    p = p - k * h * p;
    //printf("%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n", x[0], x[1], x[2], x[3], z[0], z[1], z[2], z[3]);
  }
}

