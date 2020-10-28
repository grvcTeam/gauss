//------------------------------------------------------------------------------
// GRVC 
// Author Jesus Capitan <jcapitan@us.es>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#include <tracking/target_tracker.h>
#include <ros/duration.h>
#include <iostream>

#define VEL_NOISE_VAR 0.5
#define MIN_SIZE_DISTANCE 0.15

//#define DEBUG

using namespace std;

/** Constructor
\param id Identifier
*/
TargetTracker::TargetTracker(int id): update_timer_(ros::Time(0))
{
	id_ = id;

	pose_ = Eigen::MatrixXd::Zero(4,1);
	pose_cov_ = Eigen::MatrixXd::Identity(4,4);
}

/// Destructor
TargetTracker::~TargetTracker()
{
}

/**
\brief Initialize the filter. 
\param z Initial observation
*/
void TargetTracker::initialize(Candidate* z)
{
	if (z->source == z->ADSB)
	{
		this->info_source_ = this->ADSB;
		this->icao_address_ = z->icao_address;
	}
	else if (z->source == z->POSITIONREPORT)
	{
		this->info_source_ = this->POSITIONREPORT;
		this->uav_id_ = z->uav_id;
	}

	// Setup state vector
	pose_.setZero(6, 1);
	pose_(0,0) = z->location(0);
	pose_(1,0) = z->location(1);
	pose_(2,0) = z->location(2);

	if (z->speed_available)
	{
		// Use speed information for initialization
		pose_(3,0) = z->speed(0);
		pose_(4,0) = z->speed(1);
		pose_(5,0) = z->speed(2);
		#ifdef DEBUG
		std::cout << "\n## INITIALIZE STEP USING SPEED INFO #########\n";
		#endif
	}
	else
	{
		// Don't use speed information
		pose_(3,0) = 0.0; 
		pose_(4,0) = 0.0;
		pose_(5,0) = 0.0;
		#ifdef DEBUG
		std::cout << "\n## INITIALIZE STEP NOT USING SPEED INFO ########\n";
		#endif
	}
	/*
	std::cout << "TARGET ID: " << this->id_ << "\n";
	std::cout << "position (x,y,z): " << "(" << this->pose_(0) << "," << this->pose_(1) << "," << this->pose_(2);
	std::cout << ") in meters\n";
	std::cout << "speed (x,y,z): " << "(" << this->pose_(3) << "," << this->pose_(4) << "," << this->pose_(5);
	std::cout << ") in m/s" << std::endl;
	*/

	// Setup cov matrix
	pose_cov_.setIdentity(6, 6);
	pose_cov_(0,0) = z->location_covariance(0,0); // Vxx
	pose_cov_(0,1) = z->location_covariance(0,1); // Vxy
	pose_cov_(0,2) = z->location_covariance(0,2); // Vxz

	pose_cov_(1,0) = z->location_covariance(1,0); // Vyx
	pose_cov_(1,1) = z->location_covariance(1,1); // Vyy
	pose_cov_(1,2) = z->location_covariance(1,2); // Vyz

	pose_cov_(2,0) = z->location_covariance(2,0); // Vyz
	pose_cov_(2,1) = z->location_covariance(2,1); // Vxz
	pose_cov_(2,2) = z->location_covariance(2,2); // Vzz

	// VEL_NOISE_VAR
	pose_cov_(3,3) = z->speed_covariance(0,0); // V(vxvx)
	pose_cov_(4,4) = z->speed_covariance(1,1); // V(vyvy)
	pose_cov_(5,5) = z->speed_covariance(2,2); // V(vzvz)

	// Update timer
	update_timer_.reset(z->timestamp);
	update_count_ = 0;
}

/**
\brief Predict the filter.
\param dt Length in seconds of the prediction step. 
*/
void TargetTracker::predict(ros::Time &prediction_time)
{
	// If prediction time is close to last update time or prediction time, the prediction is skipped
	double dt = update_timer_.elapsed(prediction_time).toSec();
	
	// static factors do not vary. Position depending on whether it is dynamic or not.
	#ifdef DEBUG
	std::cout << "\n## PREDICTION STEP #########\n";
	std::cout << "TARGET ID: " << this->id_ << "\n";
	std::cout << "position (x,y,z): " << "(" << this->pose_(0) << "," << this->pose_(1) << "," << this->pose_(2); 
	std::cout << ") in meters\n";
	std::cout << "speed (x,y,z): " << "(" << this->pose_(3) << "," << this->pose_(4) << "," << this->pose_(5);
	std::cout << ") in m/s" << std::endl;
	#endif
	// State vector prediction
	//std::cout << "dt: " << dt << std::endl;
	//std::cout << "reference timer: " << update_timer_.referenceTime().toSec() << std::endl;
	pose_(0,0) += pose_(3,0)*dt;
	pose_(1,0) += pose_(4,0)*dt;
	pose_(2,0) += pose_(5,0)*dt;

	// Convariance matrix prediction
	Eigen::Matrix<double, 6, 6> F;
	F.setIdentity(6, 6);
	F(0,3) = dt;
	F(1,4) = dt;
	F(2,5) = dt;

	Eigen::Matrix<double, 6, 6> Q;
	Q.setZero(6, 6);
	Q(3,3) = VEL_NOISE_VAR*dt*dt;
	Q(4,4) = VEL_NOISE_VAR*dt*dt;
	Q(5,5) = VEL_NOISE_VAR*dt*dt;

	pose_cov_ = F*pose_cov_*F.transpose() + Q;

	#ifdef DEBUG
	std::cout << "New position and speed after prediction\n";
	std::cout << "position (x,y,z): " << "(" << this->pose_(0) << "," << this->pose_(1) << "," << this->pose_(2); 
	std::cout << ") in meters\n";
	std::cout << "speed (x,y,z): " << "(" << this->pose_(3) << "," << this->pose_(4) << "," << this->pose_(5);	
	std::cout << ") in m/s" << std::endl;
	#endif

	this->update_timer_.reset(prediction_time);
}

void TargetTracker::predictNTimes(uint8_t n, double time_step, gauss_msgs::WaypointList &estimated_trajectory)
{
	gauss_msgs::Waypoint waypoint_aux;
	ros::Duration duration_aux;
	waypoint_aux.x = pose_(0,0);
	waypoint_aux.y = pose_(1,0);
	waypoint_aux.z = pose_(2,0);
	waypoint_aux.stamp = currentPositionTimestamp();

	for(int i=0; i<n; ++i)
	{
		waypoint_aux.stamp += duration_aux.fromSec((i+1)*time_step);
		waypoint_aux.x += pose_(3,0)*time_step;
		waypoint_aux.y += pose_(4,0)*time_step;
		waypoint_aux.z += pose_(5,0)*time_step;
		estimated_trajectory.waypoints.push_back(waypoint_aux);
	}
}

/**
\brief Update the filter.
\param z Observation to update. 
\return True if everything was fine
*/
bool TargetTracker::update(Candidate* z)
{
	// TODO: If a candidate with a timestamp that is back in the past with respect to the current estimator time arrives,
	// we should use it, applying some correction to the covariance, to give it less weight in the estimation.
	if ( (z->source == z->ADSB) && (this->info_source_ == this->POSITIONREPORT) )
		this->info_source_ = this->BOTH;
	else if ( (z->source == z->POSITIONREPORT) && (this->info_source_ == this->ADSB) )
		this->info_source_ = this->BOTH;

	if(!z->speed_available)
	{
		#ifdef DEBUG
		std::cout << "\n## UPDATE STEP DON'T USING SPEED INFO #########\n";
		std::cout << "TARGET ID: " << this->id_ << "\n";
		std::cout << "position (x,y,z): " << "(" << this->pose_(0) << "," << this->pose_(1) << "," << this->pose_(2);
		std::cout << ") in meters" << "\n";
		std::cout << "speed (x,y,z): " << "(" << this->pose_(3) << "," << this->pose_(4) << "," << this->pose_(5);
		std::cout << ") in m/s" << "\n";
		std::cout << "Matched candidate info\n";
		std::cout << "position (x,y,z): " << "(" << z->location(0) << "," << z->location(1) << z->location(2) << ")\n";
		std::cout << "speed (x,y,z): " << "(" << z->speed(0) << "," << z->speed(1) << "," << z->speed(2) << ")" << std::endl;
		#endif
		// Update when there are no speed measurements
		// Compute update jacobian
		Eigen::Matrix<double, 3, 6> H;
		H.setZero(3, 6);
		H(0,0) = 1.0;
		H(1,1) = 1.0;
		H(2,2) = 1.0;

		// Compute update noise matrix
		Eigen::Matrix<double, 3, 3> R;
		R(0,0) = z->location_covariance(0,0);
		R(0,1) = z->location_covariance(0,1);
		R(0,2) = z->location_covariance(0,2);
		R(1,0) = z->location_covariance(1,0);
		R(1,1) = z->location_covariance(1,1);
		R(1,2) = z->location_covariance(1,2);
		R(2,0) = z->location_covariance(2,0);
		R(2,1) = z->location_covariance(2,1);
		R(2,2) = z->location_covariance(2,2);

		// Calculate innovation matrix
		Eigen::Matrix<double, 3, 3> S;
		S = H*pose_cov_*H.transpose() + R;
		#ifdef DEBUG
		std::cout << "S matrix " << "\n";
		std::cout << S << std::endl;
		#endif

		// Calculate kalman gain
		Eigen::Matrix<double, 6, 3> K;
		K = pose_cov_*H.transpose()*S.inverse();
		#ifdef DEBUG
		std::cout << "K matrix " << "\n";
		std::cout << K << std::endl;
		#endif

		// Calculate innovation vector
		Eigen::Matrix<double, 3, 1> y;
		y = H*pose_;
		y(0,0) = z->location(0) - y(0,0);
		y(1,0) = z->location(1) - y(1,0);
		y(2,0) = z->location(2) - y(2,0);

		// Calculate new state vector
		pose_ = pose_ + K*y;

		// Calculate new cov matrix
		Eigen::Matrix<double, 6, 6> I;
		I.setIdentity(6, 6);
		pose_cov_ = (I - K*H)*pose_cov_;
	}
	else
	{
		#ifdef DEBUG
		std::cout << "\n## UPDATE STEP USING SPEED INFO #########\n";
		std::cout << "TARGET ID: " << this->id_ << "\n";
		std::cout << "position (x,y,z): " << "(" << this->pose_(0) << "," << this->pose_(1) << "," << this->pose_(2);
		std::cout << ") in meters" << "\n";
		std::cout << "speed (x,y,z): " << "(" << this->pose_(3) << "," << this->pose_(4) << "," << this->pose_(5);
		std::cout << ") in m/s" << "\n";
		std::cout << "Matched candidate info\n";
		std::cout << "position (x,y,z): " << "(" << z->location(0) << "," << z->location(1) << "," << z->location(2) << ")\n";
		std::cout << "speed (x,y,z): " << "(" << z->speed(0) << "," << z->speed(1) << "," << z->speed(2) << ")" << std::endl;
		#endif
		// Update when there are speed measurements
		// Compute update jacobian
		Eigen::Matrix<double, 6, 6> H;
		H.setIdentity(6,6);
		//std::cout << "H matrix\n";
		//std::cout << H << "\n\n";
		// Compute update noise matrix
		Eigen::Matrix<double, 6, 6> R;
		R.setZero(6,6);
		R(0,0) = z->location_covariance(0,0);
		R(0,1) = z->location_covariance(0,1);
		R(0,2) = z->location_covariance(0,2);
		R(1,0) = z->location_covariance(1,0);
		R(1,1) = z->location_covariance(1,1);
		R(1,2) = z->location_covariance(1,2);
		R(2,0) = z->location_covariance(2,0);
		R(2,1) = z->location_covariance(2,1);
		R(2,2) = z->location_covariance(2,2);

		R(3,3) = z->speed_covariance(0,0);
		R(3,4) = z->speed_covariance(0,1);
		R(3,5) = z->speed_covariance(0,2);
		R(4,3) = z->speed_covariance(1,0);
		R(4,4) = z->speed_covariance(1,1);
		R(4,5) = z->speed_covariance(1,2);
		R(5,3) = z->speed_covariance(2,0);
		R(5,4) = z->speed_covariance(2,1);
		R(5,5) = z->speed_covariance(2,2);

		//std::cout << "R matrix\n";
		//std::cout << R << "\n\n";

		// Calculate innovation matrix
		Eigen::Matrix<double, 6, 6> S;
		S = H * pose_cov_ * H.transpose() + R;
		#ifdef DEBUG
		std::cout << "S matrix\n";
		std::cout << S << "\n\n";
		#endif

		// Calculate kalman gain
		Eigen::Matrix<double, 6, 6> K;
		K = pose_cov_ * H.transpose() * S.inverse();
		#ifdef DEBUG
		std::cout << "K matrix\n";
		std::cout << K << "\n\n";
		#endif

		// Calculate innovation vector
		Eigen::Matrix<double, 6, 1> y;
		y = H * pose_;
		y(0,0) = z->location(0) - y(0,0);
		y(1,0) = z->location(1) - y(1,0);
		y(2,0) = z->location(2) - y(2,0);
		y(3,0) = z->speed(0) - y(3,0);
		y(4,0) = z->speed(1) - y(4,0);
		y(5,0) = z->speed(2) - y(5,0);

		#ifdef DEBUG
		std::cout << "y vector\n";
		std::cout << y <<"\n\n";
		#endif
		// Calculate new state vector
		pose_ = pose_ + K*y;
		
		#ifdef DEBUG
		std::cout << "New position and speed after update" << "\n";
		std::cout << "position (x,y,z): " << "(" << this->pose_(0) << "," << this->pose_(1) << "," << this->pose_(2);
		std::cout << ") in meters\n";
		std::cout << "speed (x,y,z): " << "(" << this->pose_(3) << "," << this->pose_(4) << "," << this->pose_(5);
		std::cout << ") in m/s" << std::endl;
		#endif		

		Eigen::Matrix<double, 6, 6> I;
		I.setIdentity(6,6);
		// Calculate new cov matrix
		pose_cov_ = (I - K*H)*pose_cov_;
	}

	// Update timer
	update_timer_.reset(z->timestamp);
	update_count_++;
}
    
/**
Compute the likelihood of an observation with current belief. Based on Mahalanobis distance. 
\param z Observation. 
\return Likelihood measurement
*/
double TargetTracker::getMahaDistance(Candidate* z)
{
	double distance;

	// Compute update jacobian
	Eigen::Matrix<double, 3, 6> H;
	H.setZero(4, 6);
	H(0,0) = 1.0;
	H(1,1) = 1.0;
	H(2,2) = 1.0;
		
	// Compute update noise matrix
	Eigen::Matrix<double, 3, 3> R;
	R(0,0) = z->location_covariance(0,0);
	R(0,1) = z->location_covariance(0,1);
	R(0,2) = z->location_covariance(0,2);
 
	R(1,0) = z->location_covariance(1,0);
	R(1,1) = z->location_covariance(1,1);
	R(1,2) = z->location_covariance(1,2);

	R(2,0) = z->location_covariance(2,0);
	R(2,1) = z->location_covariance(2,1);
	R(2,2) = z->location_covariance(2,2);

	// Calculate innovation matrix
	Eigen::Matrix<double, 3, 3> S;
	S = H*pose_cov_*H.transpose() + R;
			
	// Calculate innovation vector
	Eigen::Matrix<double, 3, 1> y;
	y = H*pose_;
	y(0,0) = z->location(0) - y(0,0);
	y(1,0) = z->location(1) - y(1,0);
	y(2,0) = z->location(2) - y(2,0);

	// This is a squared distance
	distance = y.transpose()*S.inverse()*y;
	// Get non squared distance
	distance = sqrt(distance);

	return distance;
}

/**
Compute the euclidean distance of an observation with current belief.
\param z Observation. 
\return Euclidean distance
*/
double TargetTracker::getDistance(Candidate* z)
{
	double dx, dy, dz;
	dx = pose_(0,0) - z->location(0);
	dy = pose_(1,0) - z->location(1);
	dz = pose_(2,0) - z->location(2);

	return sqrt(dx*dx + dy*dy + dz*dz);
}

/**
Return the time since the last observation update. 
\return Update time
*/
ros::Duration TargetTracker::lastUpdateTime(ros::Time &now)
{
	return update_timer_.elapsed(now);
}

ros::Time TargetTracker::currentPositionTimestamp()
{
	return update_timer_.referenceTime();
}

/**
Return the counter of updates. 
\return Update counter
*/
int TargetTracker::getUpdateCount()
{
	return update_count_;
}
    
/** \brief Return pose information from the target
\param x Position of the target
\param y Position of the target
*/
void TargetTracker::getPose(double &x, double &y, double &z)
{
	x = pose_(0,0);
	y = pose_(1,0);
	z = pose_(2,0);
}

/** \brief Return velocity information from the target
\param vx Velocity of the target
\param vy Velocity of the target
*/
void TargetTracker::getVelocity(double &vx, double &vy, double &vz)
{
	vx = pose_(3,0);
	vy = pose_(4,0);
	vz = pose_(5,0);
}

/** \brief Return covariance matrix from the target position
\return Covariance matrix
*/
Eigen::Matrix4d TargetTracker::getCov()
{
	return pose_cov_;
}

/** \brief Return target identifier
\return Target identifier  
*/
int TargetTracker::getId()
{
	return id_;
}