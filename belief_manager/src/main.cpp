/* 
 * File:   main.cpp
 * Author: gmilliez
 *
 * Created on February 5, 2015, 2:49 PM
 */

#include "toaster_msgs/ToasterFactReader.h"
#include "toaster_msgs/ToasterObjectReader.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/AddFact.h"
#include "toaster_msgs/RemoveFact.h"
#include "toaster_msgs/GetFactValue.h"
#include "toaster_msgs/GetFacts.h"
#include "toaster_msgs/AddAgent.h"

// factList for each monitored agent
static std::map<std::string, toaster_msgs::FactList> factListMap_;

// Agents with monitored belief
static std::map<std::string, unsigned int> agentsTracked_;

static std::string mainAgentName_ = "PR2_ROBOT";
static unsigned int mainAgentId_ = 1;

// Publisher for agents model
std::map<std::string, ros::Publisher> agentModelPub_;

ros::NodeHandle* node_;

bool removeFactToAgent(unsigned int myFactId, std::string agentName) {
    factListMap_[agentName].factList.erase(factListMap_[agentName].factList.begin() + myFactId);
    return true;
}

bool removeFactToAgent(toaster_msgs::Fact myFact, std::string agentName) {
    bool removed = false;
    for (unsigned int i = 0; i < factListMap_[agentName].factList.size(); i++) {
        if ((factListMap_[agentName].factList[i].subjectName == myFact.subjectName) &&
                (factListMap_[agentName].factList[i].targetName == myFact.targetName) &&
                (factListMap_[agentName].factList[i].property == myFact.property)) {
            //we remove it:
            removeFactToAgent(i, agentName);
            removed = true;
        }
    }
    return removed;
}

bool removePropertyTypeToAgent(std::string propertyType, std::string agentName) {
    bool removed = false;
    for (unsigned int i = 0; i < factListMap_[agentName].factList.size(); i++) {
        if (factListMap_[agentName].factList[i].propertyType == propertyType) {
            //we remove it:
            removeFactToAgent(i, agentName);
            removed = true;
        }
    }
    return removed;
}

bool removeInternFactToAgent(std::string agentName) {
    bool removed = false;
    for (unsigned int i = 0; i < factListMap_[agentName].factList.size(); i++) {
        if ((factListMap_[agentName].factList[i].propertyType != "state") &&
                (factListMap_[agentName].factList[i].propertyType != "staticProperty") &&
                (factListMap_[agentName].factList[i].propertyType != "knowledge")) {
            //we remove it:
            removeFactToAgent(i, agentName);
            removed = true;
        }
    }
    return removed;
}

// TODO: check if this is really different to addFact
// Extern fact are fact from request. They are managed by an external module.

bool addExternFactToAgent(toaster_msgs::Fact myFact, double confidenceDecrease, std::string agentName) {
    // We verify that this fact is not already there.
    for (unsigned int i = 0; i < factListMap_[agentName].factList.size(); i++) {
        if ((factListMap_[agentName].factList[i].subjectName == myFact.subjectName) &&
                (factListMap_[agentName].factList[i].targetName == myFact.targetName) &&
                (factListMap_[agentName].factList[i].property == myFact.property)) {
            // as it is the same fact, we remove the previous value:
            removeFactToAgent(i, agentName);
            printf("[BELIEF_MANAGER][WARNING] Fact added to agent %s was already in "
                    "current fact list: \n fact %s %s %s was removed to avoid double\n",
                    agentName.c_str(), factListMap_[agentName].factList[i].subjectName.c_str(),
                    factListMap_[agentName].factList[i].property.c_str(),
                    factListMap_[agentName].factList[i].targetName.c_str());
        }
    }
    myFact.confidence *= confidenceDecrease;
    factListMap_[agentName].factList.push_back(myFact);
    return true;
}

// When adding a fact to an agent, the confidence may decrease as
// the other's belief are suppositions based on observation

bool addFactToAgent(toaster_msgs::Fact myFact, double confidenceDecrease, std::string agentName) {
    // We verify that this fact is not already there.
    for (unsigned int i = 0; i < factListMap_[agentName].factList.size(); i++) {
        if ((factListMap_[agentName].factList[i].subjectName == myFact.subjectName) &&
                (factListMap_[agentName].factList[i].targetName == myFact.targetName) &&
                (factListMap_[agentName].factList[i].property == myFact.property)) {
            // as it is the same fact, we remove the previous value:
            removeFactToAgent(i, agentName);
            printf("[BELIEF_MANAGER][WARNING] Fact added to agent %s was already in "
                    "current fact list: \n fact %s %s %s was removed to avoid double\n",
                    agentName.c_str(), factListMap_[agentName].factList[i].subjectName.c_str(),
                    factListMap_[agentName].factList[i].property.c_str(),
                    factListMap_[agentName].factList[i].targetName.c_str());
        }
    }
    myFact.confidence *= confidenceDecrease;
    factListMap_[agentName].factList.push_back(myFact);
    return true;
}

bool getFactValueFromAgent(toaster_msgs::Fact reqFact, std::string name, toaster_msgs::Fact& resFact) {
    // Find fact:
    for (std::vector<toaster_msgs::Fact>::iterator itFact = factListMap_[name].factList.begin(); itFact != factListMap_[name].factList.end(); ++itFact) {
        if ((*itFact).property == reqFact.property
                && (*itFact).subjectName == reqFact.subjectName
                && (*itFact).targetName == reqFact.targetName) {
            resFact = (*itFact);
            return true;
        } else {
            continue;
        }
    }
    ROS_INFO("[agent_monitor][gatFactValue][WARNING] Fact requested was not found in agent %s model\n", name.c_str());
    return false;
}

bool getFactsFromAgent(toaster_msgs::Fact reqFact, std::string name, toaster_msgs::FactList& resFactList) {
    // Find fact:
    for (std::vector<toaster_msgs::Fact>::iterator itFact = factListMap_[name].factList.begin(); itFact != factListMap_[name].factList.end(); ++itFact) {

        // We verify first the property:
        if (reqFact.property == "" || (*itFact).property == reqFact.property)
            if (reqFact.targetName == "" || (*itFact).targetName == reqFact.targetName)
                if (reqFact.subjectName == "" || (*itFact).subjectName == reqFact.subjectName)
                    if (reqFact.subjectId == 0 || (*itFact).subjectId == reqFact.subjectId)
                        if (reqFact.targetId == 0 || (*itFact).targetId == reqFact.targetId)
                            if (reqFact.propertyType == "" || (*itFact).propertyType == reqFact.propertyType)
                                if (reqFact.subProperty == "" || (*itFact).subProperty == reqFact.subProperty)
                                    if (reqFact.stringValue == "" || (*itFact).stringValue == reqFact.stringValue)
                                        if (reqFact.doubleValue == 0.0 || (*itFact).doubleValue == reqFact.doubleValue)
                                            if (reqFact.confidence == 0.0 || (*itFact).confidence == reqFact.confidence)
                                                resFactList.factList.push_back((*itFact));

    }
    if (resFactList.factList.size() == 0) {
        ROS_INFO("[agent_monitor][gatFacts][WARNING] Fact requested was not found in agent %s model\n", name.c_str());
        return false;
    } else {
        return true;
    }
}

void addAgentModel(unsigned int id, std::string name) {
    //If no publisher yet
    if (agentModelPub_.find(name) == agentModelPub_.end()) {
        std::stringstream ss;
        ss << "belief_manager/" << name << "/factList";
        //factList publisher
        agentModelPub_[name] = node_->advertise<toaster_msgs::FactList>(ss.str(), 1000);
    }
    if (agentsTracked_.find(name) == agentsTracked_.end()) {
        agentsTracked_[name] = id;
    }
}


//////////////
// Services //
//////////////

bool getFactValue(toaster_msgs::GetFactValue::Request &req,
        toaster_msgs::GetFactValue::Response & res) {

    std::string name = "";

    if (req.agentName != "")
        name = req.agentName;
    else if (req.agentId != 0)
        if (req.agentId == mainAgentId_)
            name = mainAgentName_;
            //else if (agentsTracked_.find(req.agentId) != agentsTracked_.end())
            //    name = agentsTracked_[req.agentId];
        else
            ROS_INFO("[agent_monitor][request][WARNING] Request to get fact value in %s model who is untracked agent\n", req.agentName.c_str());
    else {
        ROS_INFO("[agent_monitor][request][WARNING] Request to get fact value in without agent model specified. We will look in main agent belief state\n");
        name = mainAgentName_;
    }
    if (name != "") {
        res.boolAnswer = getFactValueFromAgent(req.reqFact, name, res.resFact);
        return true;
    } else
        return false;
}

bool getFacts(toaster_msgs::GetFacts::Request &req,
        toaster_msgs::GetFacts::Response & res) {

    std::string name = "";

    if (req.agentName != "")
        name = req.agentName;
    else if (req.agentName != "")
        if (req.agentId == mainAgentId_)
            name = mainAgentName_;
            //else if (agentsTracked_.find(req.agentName) != agentsTracked_.end())
            //  id = agentsTracked_[req.agentName];
        else
            ROS_INFO("[agent_monitor][request][WARNING] Request to get fact value in %s model who is untracked agent\n", req.agentName.c_str());
    else {
        ROS_INFO("[agent_monitor][request][WARNING] Request to get fact value in without agent model specified. We will look in main agent belief state\n");
        name = mainAgentName_;
    }
    if (name != "") {
        res.boolAnswer = getFactsFromAgent(req.reqFact, name, res.resFactList);
        return true;
    } else
        return false;
}

bool addFact(toaster_msgs::AddFact::Request &req,
        toaster_msgs::AddFact::Response & res) {

    /**************************/
    /* World State management */
    /**************************/

    // Add safely the fact to main agent
    res.answer = addExternFactToAgent(req.fact, 1.0, mainAgentName_);

    // We update an agent belief state if he is in same room, has visibility on subject
    // and we don't care here about the observability as we assess that the agent saw the action.

    /**********************************/
    /* Conceptual perspective taking: */
    /**********************************/


    // TODO: for each agent present and in same room
    for (std::map<std::string, unsigned int>::iterator it = agentsTracked_.begin(); it != agentsTracked_.end(); ++it) {
        for (std::vector<toaster_msgs::Fact>::iterator itFactVisibility = factListMap_[mainAgentName_].factList.begin(); itFactVisibility != factListMap_[mainAgentName_].factList.end(); ++itFactVisibility) {

            if ((*itFactVisibility).property == "IsVisible"
                    // Current agent
                    && (*itFactVisibility).subjectName == it->first
                    // has visibility
                    && (*itFactVisibility).targetName
                    // On current fact subject
                    == req.fact.subjectName) {

                addExternFactToAgent(req.fact, (*itFactVisibility).doubleValue, it->first);
            }
        }
    }
    ROS_INFO("request: adding a new fact");
    ROS_INFO("sending back response: [%d]", (int) res.answer);
    return true;
}

bool removeFact(toaster_msgs::RemoveFact::Request &req,
        toaster_msgs::RemoveFact::Response & res) {

    /**************************/
    /* World State management */
    /**************************/

    // Remove safely the fact to main agent
    res.answer = removeFactToAgent(req.fact, mainAgentName_);


    /**********************************/
    /* Conceptual perspective taking: */
    /**********************************/


    // TODO: for each agent with visibility on subject
    for (std::map<std::string, unsigned int>::iterator it = agentsTracked_.begin(); it != agentsTracked_.end(); ++it) {
        for (std::vector<toaster_msgs::Fact>::iterator itFactVisibility = factListMap_[mainAgentName_].factList.begin(); itFactVisibility != factListMap_[mainAgentName_].factList.end(); ++itFactVisibility) {

            if ((*itFactVisibility).property == "IsVisible"
                    // Current agent
                    && (*itFactVisibility).subjectName == it->first
                    // has visibility
                    && (*itFactVisibility).targetName
                    // On current fact subject
                    == req.fact.subjectName) {
                removeFactToAgent(req.fact, it->first);
            }
        }
    }
    ROS_INFO("request: removing a fact");
    ROS_INFO("sending back response: [%d]", (int) res.answer);
    return true;

}

bool publishAgentModel(toaster_msgs::AddAgent::Request &req,
        toaster_msgs::AddAgent::Response & res) {
    addAgentModel(req.id, req.name);
    res.answer = true;
    return true;
}



/********************************************************/
/**                     MAIN LOOP                      **/

/********************************************************/


int main(int argc, char** argv) {
    // Set this in a ros service
    ros::init(argc, argv, "belief_manager");
    ros::NodeHandle node;
    node_ = &node;

    //Data reading
    ToasterFactReader factRdSpark(node, "spark/factList");
    ToasterFactReader factRdPdg(node, "pdg/factList");
    ToasterFactReader factRdArea(node, "area_manager/factList");
    ToasterFactReader factRdAM(node, "agent_monitor/factList");
    //ToasterObjectReader objectRd(node);

    //Services
    ros::ServiceServer serviceAdd = node.advertiseService("belief_manager/add_fact", addFact);
    ROS_INFO("Ready to add fact.");

    ros::ServiceServer serviceRemove = node.advertiseService("belief_manager/remove_fact", removeFact);
    ROS_INFO("Ready to remove fact.");

    ros::ServiceServer serviceGetFactValue = node.advertiseService("belief_manager/get_fact_value", getFactValue);
    ROS_INFO("Ready to get fact value.");

    ros::ServiceServer serviceGetFacts = node.advertiseService("belief_manager/get_facts", getFacts);
    ROS_INFO("Ready to get facts.");

    ros::ServiceServer serviceAddAgentModel = node.advertiseService("belief_manager/add_agent_to_track", publishAgentModel);
    ROS_INFO("Ready to track agent belief.");


    static ros::Publisher fact_pub_main = node.advertise<toaster_msgs::FactList>("belief_manager/factList", 1000);

    // Set this in a ros service?
    ros::Rate loop_rate(30);

    // Vector of Objects.
    //std::map<std::string, Object*> mapObject;


    /************************/
    /* Start of the Ros loop*/
    /************************/

    while (node.ok()) {


        /**************************/
        /* World State management */
        /**************************/

        // We remove intern fact for main agent
        removeInternFactToAgent(mainAgentName_);
        // First we feed the mainAgent belief state
        for (unsigned int i = 0; i < factRdArea.lastMsgFact.factList.size(); i++) {
            addFactToAgent(factRdArea.lastMsgFact.factList[i], 1.0, mainAgentName_);
        }

        for (unsigned int i = 0; i < factRdSpark.lastMsgFact.factList.size(); i++) {
            addFactToAgent(factRdSpark.lastMsgFact.factList[i], 1.0, mainAgentName_);
        }

        for (unsigned int i = 0; i < factRdPdg.lastMsgFact.factList.size(); i++) {
            addFactToAgent(factRdPdg.lastMsgFact.factList[i], 1.0, mainAgentName_);
        }

        for (unsigned int i = 0; i < factRdAM.lastMsgFact.factList.size(); i++) {
            addFactToAgent(factRdAM.lastMsgFact.factList[i], 1.0, mainAgentName_);
        }


        /**********************************/
        /* Conceptual perspective taking: */
        /**********************************/

        // We remove facts that are visible before updating.
        for (std::map<std::string, unsigned int>::iterator itAgent = agentsTracked_.begin(); itAgent != agentsTracked_.end(); ++itAgent) {
            for (std::vector<toaster_msgs::Fact>::iterator itFactAgent = factListMap_[itAgent->first].factList.begin(); itFactAgent != factListMap_[itAgent->first].factList.end(); ++itFactAgent) {
                // Update if:
                // 1) fact is observable
                if ((*itFactAgent).factObservability > 0.0) {

                    // 2) Agent has visibility on fact subject
                    for (std::vector<toaster_msgs::Fact>::iterator itFactVisibility = factListMap_[mainAgentName_].factList.begin(); itFactVisibility != factListMap_[mainAgentName_].factList.end(); ++itFactVisibility) {

                        //TODO: use function getFact
                        if (((*itFactVisibility).property == "IsVisible"
                                // Current agent
                                && (*itFactVisibility).subjectName == itAgent->first
                                // has visibility
                                && (*itFactVisibility).targetName
                                // On current fact subject
                                == (*itFactAgent).subjectName)
                                // Or is himself the current fact subject
                                || ((*itFactAgent).subjectName == itAgent->first)) {


                            /*printf("Agent %s has visibility on %s. We remove related fact to agent model before update"
                                    "\n fact added to %s: %s %s %s \n",
                                    it->first.c_str(),
                                    factListMap_[mainAgentName_].factList[i].subjectName.c_str(),
                                    it->first.c_str(),
                                    factListMap_[mainAgentName_].factList[i].subjectName.c_str(),
                                    factListMap_[mainAgentName_].factList[i].property.c_str(),
                                    factListMap_[mainAgentName_].factList[i].targetName.c_str());

                             */
                            removeFactToAgent((*itFactAgent), itAgent->first);
                        }
                    }
                    //Or if fact concerns himself
                } else if ((*itFactAgent).subjectName == itAgent->first) {
                    removeFactToAgent((*itFactAgent), itAgent->first);
                }
            }


            // Agent observes new facts
            for (unsigned int i = 0; i < factListMap_[mainAgentName_].factList.size(); i++) {
                // Update if:
                // 1) fact is observable
                if (factListMap_[mainAgentName_].factList[i].factObservability > 0.0) {

                    // 2) Agent has visibility on fact subject
                    for (std::vector<toaster_msgs::Fact>::iterator itFactVisibility = factListMap_[mainAgentName_].factList.begin(); itFactVisibility != factListMap_[mainAgentName_].factList.end(); ++itFactVisibility) {
                        if (((*itFactVisibility).property == "IsVisible"
                                // Current agent
                                && (*itFactVisibility).subjectName == itAgent->first
                                // has visibility
                                && (*itFactVisibility).targetName
                                // On current fact subject
                                == factListMap_[mainAgentName_].factList[i].subjectName)
                                // Or is himself the current fact subject
                                || (factListMap_[mainAgentName_].factList[i].subjectName == itAgent->first)) {


                            //printf("Agent %s has visibility on %s. We add related fact to agent model "
                            //        "\n fact added to %s: %s %s %s \n",
                            //        it->first.c_str(),
                            //        factListMap_[mainAgentName_].factList[i].subjectName.c_str(),
                            //        it->first.c_str(),
                            //        factListMap_[mainAgentName_].factList[i].subjectName.c_str(),
                            //        factListMap_[mainAgentName_].factList[i].property.c_str(),
                            //        factListMap_[mainAgentName_].factList[i].targetName.c_str());


                            if ((factListMap_[mainAgentName_].factList[i].propertyType != "state") &&
                                    (factListMap_[mainAgentName_].factList[i].propertyType != "staticProperty") &&
                                    (factListMap_[mainAgentName_].factList[i].propertyType != "knowledge"))
                                addFactToAgent(factListMap_[mainAgentName_].factList[i], (*itFactVisibility).doubleValue * factListMap_[mainAgentName_].factList[i].factObservability, itAgent->first);
                            else
                                addExternFactToAgent(factListMap_[mainAgentName_].factList[i], (*itFactVisibility).doubleValue * factListMap_[mainAgentName_].factList[i].factObservability, itAgent->first);
                        }
                    }
                }
            }
        }

        //TODO: publish for each agent:
        fact_pub_main.publish(factListMap_[mainAgentName_]);
        for (std::map<std::string, ros::Publisher>::iterator it = agentModelPub_.begin(); it != agentModelPub_.end(); ++it) {
            it->second.publish(factListMap_[it->first]);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}

