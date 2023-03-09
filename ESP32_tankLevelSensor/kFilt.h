/**   KalmanFilter object
 *    create a Kalman filter with custum parameters, and filter
 *    the sensor reading
 */
class KalmanFilter{
  private:
    double R = 10;   /// measurement noise
    double H = 1;

    double Q = 0.1;   /// initial covariance
    double P = 0;    /// initial error covariance
    double K = 0;    /// initial Kalman gain

    const double hist = 5;
  public:
    double distBar = -1;     /// estimated variable

  KalmanFilter(){            /// empty constructor
   // this->R = 1;
   // this->H = 10;
    //this->R = R;
    //this->H = H;
  }
  void setRH(double R, double H){   /// set parameters of Kalman filter
    this->R = R;
    this->H = H;
  }
  double getFilt(double dist){     /// get filtered
    if (distBar == -1) {           /// if this is the first reading, do not filter
      distBar = dist;
    }
    if(abs(distBar - dist) < hist){   /// if the difference between estimated and real is less than hist, do not filter
      return distBar;                 ///return previous filtered
    }

    K = P * H / (H * H * P + R);                      /// do the Kalman filter magic.
    distBar = distBar + K * (dist - H * distBar);
    P = (1 - K * H) * P + Q;

    return distBar;                   /// return filtered
  }
};
