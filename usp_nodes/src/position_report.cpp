#include <ros/ros.h>
#include <mavros_msgs/ADSBVehicle.h>
#include <gauss_msgs/RPA_state.h>
#include <gauss_msgs/PositionReport.h>


// Class definition
class positionReport
{
public:
    positionReport();

private:
    // Topic Callbacks
    void adsbCB(const mavros_msgs::ADSBVehicle::ConstPtr& msg);             // TODO: To check how ADSB information is read
    void RPAStatusCB(const gauss_msgs::RPA_state::ConstPtr& msg);        // TBC


    // Auxilary variables
    gauss_msgs::PositionReport position;


    ros::NodeHandle nh_;


    // Subscribers
    ros::Subscriber adsb_sub_;
    ros::Subscriber rpaStatus_sub_;

    // Publisher
    ros::Publisher position_pub_;

};

// positionReport Constructor
positionReport::positionReport()
{
    // Read parameters
    //nh_.param("desired_altitude",desired_altitude,0.5);


    // Initialization


    // Publish
    position_pub_=nh_.advertise<gauss_msgs::PositionReport>("/gauss/position_report",1);

    // Subscribe
    adsb_sub_=nh_.subscribe<mavros_msgs::ADSBVehicle>("/gauss/adsb_rx",1,&positionReport::adsbCB,this);
    rpaStatus_sub_=nh_.subscribe<gauss_msgs::RPA_state>("/gauss/rpa_state",1,&positionReport::RPAStatusCB,this);


    ROS_INFO("Started Position Report node!");
}



// ADSB callback
void positionReport::adsbCB(const mavros_msgs::ADSBVehicle::ConstPtr &msg)
{
    position.header=msg->header;
    position.rpa_state.position.header=msg->header;
    position.rpa_state.position.latitude=msg->latitude;
    position.rpa_state.position.longitude=msg->longitude;
    position.rpa_state.position.altitude_msl=msg->altitude;
    position.source = position.SOURCE_ADSB;
    position.rpa_state.id = msg->ICAO_address;
    position.confidence=1.0;
    // TBC

    position_pub_.publish(position);
}

// RPAStatus Callback
void positionReport::RPAStatusCB(const gauss_msgs::RPA_state::ConstPtr &msg)
{
    position.header=msg->header;
    position.confidence=1.0; // it could be estimated based on hdop ¿?
    position.rpa_state=*msg;
    position.source=position.SOURCE_RPA;
    // TBC

    position_pub_.publish(position);
}


// MAIN function
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"positionReport");

    // Create a Position_reporting object
    positionReport *position_reporting = new positionReport();

    ros::spin();
}
