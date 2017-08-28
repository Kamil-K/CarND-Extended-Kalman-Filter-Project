#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  ekf_.R_laser = MatrixXd(2,2);
  ekf_.R_radar = MatrixXd(3,3);
  //ekf_.H_laser = MatrixXd(2,4);
  //ekf_.Hj_ = MatrixXd(3,4);
  ekf_.H_ = MatrixXd(2,4);
  ekf_.Hj = MatrixXd(3,4);

  //measurement covariance matrix - laser
  ekf_.R_laser << 0.0225, 0,
			 0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_radar << 0.09, 0, 0,
			 0, 0.0009, 0,
			 0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // process noise is Q and measurement noise is R
  
  //measurement matrix
  //ekf_.H_laser << 1, 0, 0, 0,
  ekf_.H_ << 1, 0, 0, 0,
			 0, 1, 0, 0;
		
  //Jacobian matrix		
  //ekf_.Hj_ << 0, 0, 0, 0,
  ekf_.Hj << 0, 0, 0, 0,
		 0, 0, 0, 0,
	     0, 0, 0, 0;
  
  //state covariance matrix P
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
	  	0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
			
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
			  		
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		1, 0, 1, 0,
		0, 1, 0, 1;
  
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 0, 0, 0, 0;
  
  //ekf_.Init(x_, P_, F_, H_laser, Hj_, R_laser, R_radar, Q_);
				
	/*
	MatrixXd F_ = MatrixXd(4,4);
	//the initial transition matrix F_
	F_ << 1, 0, 1, 0,
		  0, 1, 0, 1,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
		  
	MatrixXd Q_ = MatrixXd(4,4);
	*/
	
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //QUESTION: what else should be initialised? Is only noise okay?
  
  //H_laser_ << 1, 0, 0, 0,
  //			0, 1, 0, 0;
			
  //Hj_ << 
  //to be initialized:
  //H_laser_ = ???;
  //Hj_ = ???; //H_jacobian

  /*
  H_laser_ << 0, 0,
			0, 0,
			0, 0,
			0, 0;
			
  Hj_ << 0, 0, 0,
		0, 0, 0, 
		0, 0, 0, 
		0, 0, 0;
	*/	
	//noise_ax = 9;
	//noise_ay = 9;
	/*
	float noise_ax = 9;
	float noise_ay = 9;
	*/
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
		/**
		TODO:
		  * Initialize the state ekf_.x_ with the first measurement.
		  * Create the covariance matrix.
		  * Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 1, 1, 1, 1;
		
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		  /**
		  Convert radar from polar to cartesian coordinates and initialize state.
		  */
		
			float px_init=measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]);
			float py_init=-measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]);

			ekf_.x_ << px_init, py_init, 0, 0;

			//previous_timestamp_ = measurement_pack.raw_measurements_[4];
			previous_timestamp_ = measurement_pack.timestamp_;
			
		
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
		  /**
		  Initialize state.
		  */
			ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

			//previous_timestamp_ = measurement_pack.raw_measurements_[3];
			previous_timestamp_ = measurement_pack.timestamp_;
		  
		}

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   //Time
   	float dt = measurement_pack.timestamp_ - previous_timestamp_;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	//previous_timestamp_ = measurement_pack.raw_measurements_[3];
   
    //pay attention if all elements of F are declared and initialized
   	//kf_.F_(0, 2) = dt;
	//kf_.F_(1, 3) = dt;
	
	/*
	//MatrixXd ekf_.F_ = MatrixXd(4,4);
	ekf_.F_ = MatrixXd(4,4);
	//the initial transition matrix F_
	ekf_.F_ << 1, 0, 1, 0,
		  0, 1, 0, 1,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
	*/
			  
	//F_ = MatrixXd(4,4);
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
   
   //process noise covariance matrix
   //R = ???;
      
   // Q covariance matrix
   
  
   	//set the acceleration noise components
	float noise_ax = 9;
	float noise_ay = 9;
	
	
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	
	//set the process covariance matrix Q
	//kf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
	
	
	//MatrixXd ekf_.Q_ = MatrixXd(4,4);
	//ekf_.Q_ = MatrixXd(4,4);
	
	//Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
	
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
