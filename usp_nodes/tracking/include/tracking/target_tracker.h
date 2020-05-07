//------------------------------------------------------------------------------
// GRVC 
// Author Jesus Capitan <jcapitan@us.es>
//------------------------------------------------------------------------------
//
// Copyright (c) 2016 GRVC University of Seville
//
//------------------------------------------------------------------------------

#ifndef TARGET_TRACKER_H_
#define TARGET_TRACKER_H_

#include <tracking/candidate.h>
#include <tracking/timer.hpp>
#include <gauss_msgs/Operation.h>

#include <vector>
#include <Eigen/Eigen>
   
/** \brief This class implements a stochastic filter for an object. 

This class implements a stochastic filter for an object. The filter estimates some continuous features 
(e.g., the position and velocity) and some discrete features (e.g., size). The object may be static or 
moving. 

*/

class TargetTracker 
{
public:
	TargetTracker(int id);
	~TargetTracker();

	void initialize(Candidate* z);
	void predict(ros::Time &prediction_time);
	void predictNTimes(uint8_t n, double time_step, gauss_msgs::WaypointList &estimated_trajectory);
	bool update(Candidate* z);
	double getMahaDistance(Candidate* z);
	double getDistance(Candidate* z);
	ros::Duration lastUpdateTime(ros::Time &now);
	ros::Time currentPositionTimestamp();
	int getUpdateCount();
	void getPose(double &x, double &y, double &z);
	void getVelocity(double &vx, double &vy, double &vz);
	Eigen::Matrix4d getCov();

	int getId();

protected:
	Timer update_timer_;			/// Timer for last update or last prediction
	int update_count_;				/// Counter with the number of updates
	int id_;						/// Target identifier

	std::string icao_address_;
	uint8_t uav_id_;

	bool non_cooperative_;

	enum InfoSource {ADSB=0, POSITIONREPORT=1, BOTH=2};
	InfoSource info_source_;

	/// State vector: [x (m), y (m), vx (m/s), vy (m/s)]
	Eigen::MatrixXd pose_;
	Eigen::MatrixXd pose_cov_;
};

#endif
