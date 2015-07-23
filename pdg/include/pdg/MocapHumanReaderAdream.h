//This class read topic from mocap and convert data into toaster-lib type.

#include "HumanReader.h"

#include <ros/ros.h>
#include "tf/transform_listener.h"
#include <string>
#include "geometry_msgs/PoseStamped.h"
#include "optitrack/or_pose_estimator_state.h"
#include "tf/transform_listener.h"
#include <sys/time.h>
#include <math.h>
#include <ostream>

class MocapHumanReaderAdream : public HumanReader {
public:
    MocapHumanReaderAdream(ros::NodeHandle& node, std::string topicHead, std::string topicHand);

private:
    ros::Subscriber subHead_;
    ros::Subscriber subHand_;
    void optitrackCallbackHead(const optitrack::or_pose_estimator_state::ConstPtr& msg);
    void optitrackCallbackHand(const optitrack::or_pose_estimator_state::ConstPtr& msg);
};
