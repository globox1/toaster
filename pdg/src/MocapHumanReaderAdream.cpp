// A human reader is a class that will read data from human(s)


#include "pdg/MocapHumanReaderAdream.h"

MocapHumanReaderAdream::MocapHumanReaderAdream(ros::NodeHandle& node, std::string topicHead, std::string topicHand) {
    std::cout << "Initializing MocapHumanReaderAdream" << std::endl;
    // ******************************************
    // Starts listening to the joint_states
    fullHuman_ = false;
    subHead_ = node.subscribe(topicHead, 1, &MocapHumanReaderAdream::optitrackCallbackHead, this);
    subHand_ = node.subscribe(topicHand, 1, &MocapHumanReaderAdream::optitrackCallbackHand, this);
    std::cout << "Done\n";
}

/*
  Gets data from a TrackedPersons msg in the human map. This msg contains a list of agens with
  their positions and orientations.
 */
void MocapHumanReaderAdream::optitrackCallbackHead(const optitrack::or_pose_estimator_state::ConstPtr& msg) {
 
    ros::Time now = ros::Time::now();
    Human* curHuman;	

    try {
            int humId = 101;
            //create a new human with the same id as the message
            if(lastConfig_[humId] == NULL){
            curHuman = new Human(humId);
            curHuman->setName("HERAKLES_HUMAN1");
            }else{
                curHuman = lastConfig_[humId];
            }

            if(msg->pos.size() != 0){
            //set human position
            bg::model::point<double, 3, bg::cs::cartesian> humanPosition;
            humanPosition.set<0>(msg->pos[0].x + 6.407);
            humanPosition.set<1>(msg->pos[0].y + 2.972);
            humanPosition.set<2>(msg->pos[0].z - 0.4);

            //set the human orientation
            std::vector<double> humanOrientation;

            //transform the pose message
            humanOrientation.push_back(0.0);
            humanOrientation.push_back(0.0);
            humanOrientation.push_back(msg->pos[0].qz);

            //put the data in the human
            curHuman->setOrientation(humanOrientation);
            curHuman->setPosition(humanPosition);
            curHuman->setTime(now.toNSec());

            lastConfig_[humId] = curHuman;
	}
      
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());


    }
}

void MocapHumanReaderAdream::optitrackCallbackHand(const optitrack::or_pose_estimator_state::ConstPtr& msg) {

    ros::Time now = ros::Time::now();
    Human* curHuman;
    Joint* curJoint;

    try {

	int humId = 101;
	std::string jointName = "rightHand";

	if(lastConfig_[humId] == NULL){
            curHuman = new Human(humId);
            curHuman->setName("HERAKLES_HUMAN1");
        }else{
               curHuman = lastConfig_[humId];
        }
	
	if(curHuman->skeleton_[jointName] == NULL){
	      curJoint = new Joint(10101, 101);
	      curJoint->setName("rightHand");
	}else{
	      curJoint = curHuman->skeleton_[jointName];
	}

	if(msg->pos.size() != 0){	
	bg::model::point<double, 3, bg::cs::cartesian> jointPosition;
        jointPosition.set<0>(msg->pos[0].x + 6.4329);
        jointPosition.set<1>(msg->pos[0].y + 3.2017);
        jointPosition.set<2>(msg->pos[0].z);
            
        std::vector<double> jointOrientation;

        jointOrientation.push_back(0.0);
        jointOrientation.push_back(0.0);
        jointOrientation.push_back(msg->pos[0].qz);


	curJoint->setPosition(jointPosition);
	curJoint->setOrientation(jointOrientation);
	curJoint->setTime(now.toNSec());

	lastConfig_[humId]->skeleton_[jointName] = curJoint;
	}

    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());

    }
}
