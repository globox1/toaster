// This main will compute the facts for the requested agent.

#include "SPAR/PDGHumanReader.h"
#include "SPAR/PDGRobotReader.h"
#include "SPAR/PDGObjectReader.h"
#include "PDG/FactList.h"
#include "PDG/Fact.h"
#include "toaster-lib/TRBuffer.h"
#include "toaster-lib/MathFunctions.h"

bool computeMotion2D(TRBuffer< Entity* > confBuffer, unsigned long timelapse, double distanceThreshold) {
    int index;
    double dist = 0.0;
    long actualTimelapse = 0;
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;
    Entity* entNew = confBuffer.getDataFromIndex(confBuffer.size() - 1);

    index = confBuffer.getIndexAfter(timeOld);
    // In case we don't have the index, we will just put isMoving to false
    if(index == -1)
      return false;
    actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index); // Actual timelapse
    Entity* entOld = confBuffer.getDataFromIndex(index);

    dist = bg::distance(MathFunctions::convert3dTo2d(entNew->getPosition()), MathFunctions::convert3dTo2d(entOld->getPosition()));

    /*std::cout << "Distance is " << dist << std::endl;
    std::cout << "ds*actualTimeLapse / timelapse " << distanceThreshold * actualTimelapse / timelapse << std::endl;
    std::cout << "ds " << distanceThreshold << std::endl;
    std::cout << "timelapse " << timelapse << std::endl;
    std::cout << "actual timelapse " << actualTimelapse << std::endl;*/
    if (dist < distanceThreshold * actualTimelapse / timelapse) {
        return false;
    } else {
        return true;
    }
}

double computeMotion2DDirection(TRBuffer< Entity* > confBuffer, unsigned long timelapse) {
    double towardAngle;
    int index;
    //long actualTimelapse = 0;
    long timeNew = confBuffer.getTimeFromIndex(confBuffer.size() - 1);
    long timeOld = timeNew - timelapse;
    Entity* entNew = confBuffer.getDataFromIndex(confBuffer.size() - 1);

    index = confBuffer.getIndexAfter(timeOld);
    //actualTimelapse = timeNew - confBuffer.getTimeFromIndex(index);   // Actual timelapse
    Entity* entOld = confBuffer.getDataFromIndex(index);
    towardAngle = acos(fabs(entOld->getPosition().get<0>() - entNew->getPosition().get<0>())
            / bg::distance(MathFunctions::convert3dTo2d(entOld->getPosition()), MathFunctions::convert3dTo2d(entNew->getPosition())));

    // Trigonometric adjustment
    if (entNew->getPosition().get<0>() < entOld->getPosition().get<0>())
        towardAngle = 3.1416 - towardAngle;

    if (entNew->getPosition().get<1>() < entOld->getPosition().get<1>())
        towardAngle = -towardAngle;

    return towardAngle;
}

std::map<unsigned int, double> computeMotion2DToward(std::map<unsigned int, TRBuffer < Entity* > > mapEnts,
        unsigned int agentMonitored, double towardAngle, double angleThreshold) {

    std::map<unsigned int, double> towardConfidence;

    //For each entities in the same room
    for (std::map<unsigned int, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        if (it->first != agentMonitored)
            towardConfidence[it->first] = MathFunctions::isInAngle(mapEnts[agentMonitored].back(), it->second.back(), towardAngle, angleThreshold);
    }
    return towardConfidence;
}

std::map<unsigned int, double> computeDeltaDist(std::map<unsigned int, TRBuffer < Entity* > > mapEnts, unsigned int agentMonitored, unsigned long timelapse) {
    std::map<unsigned int, double> deltaDistMap;
    double curDist = 0.0;
    double prevDist = 0.0;
    double deltaDist = 0.0;
    unsigned long timeCur = 0;
    unsigned long timePrev = 0;
    Entity * entCur(0);
    Entity * entMonitoredCur(0);
    Entity * entMonitoredPrev(0);

    //For each entities in the same room
    for (std::map<unsigned int, TRBuffer < Entity*> >::iterator it = mapEnts.begin(); it != mapEnts.end(); ++it) {
        if (it->first != agentMonitored) {
            // We compute the current distance
            entCur = it->second.back();
            entMonitoredCur = mapEnts[agentMonitored].back();

            //Put this in a function
            curDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()), MathFunctions::convert3dTo2d(entMonitoredCur->getPosition()));

            // We compute the distance at now - timelapse
            timeCur = entMonitoredCur->getTime();
            timePrev = timeCur - timelapse;

            entMonitoredPrev = mapEnts[agentMonitored].getDataFromIndex(mapEnts[agentMonitored].getIndexAfter(timePrev));

            prevDist = bg::distance(MathFunctions::convert3dTo2d(entCur->getPosition()), MathFunctions::convert3dTo2d(entMonitoredPrev->getPosition()));


            //We compute Deltadist
            deltaDist = curDist - prevDist;

            // We fill towardConfidence
            deltaDistMap[it->first] = deltaDist;
        }
    }
    return deltaDistMap;
}

void initTRBuffer(unsigned int agentMonitored, TRBuffer<Entity*>& TRBEntity, unsigned int historyLength) {
    //We need to initiate the ringbuffer

    TRBuffer<Entity*> mybuffer(historyLength);
    TRBEntity = mybuffer;
}

int main(int argc, char** argv) {
    // Set this in a ros service
    const bool AGENT_FULL_CONFIG = false; //If false we will use only position and orientation
    unsigned int roomOfInterest = 0;

    // Set this in a ros service
    // TODO: Make them vectors?
    unsigned int agentMonitored = 101;
    std::string jointMonitoredName = "rWristX";
    unsigned int jointMonitoredId = 0;
    bool humanMonitored = agentMonitored - 100;

    // Map of Timed Ring Buffer Entities
    static std::map<unsigned int, TRBuffer < Entity* > > mapTRBEntity;
    std::map<unsigned int, TRBuffer < Entity* > >::iterator itTRB;

    ros::init(argc, argv, "AGENT_MONITOR");
    ros::NodeHandle node;

    // TODO: add SPAR data reading to get the room of entities.
    //Data reading
    PDGHumanReader humanRd(node, true);
    PDGRobotReader robotRd(node, AGENT_FULL_CONFIG);
    PDGObjectReader objectRd(node);

    ros::Publisher fact_pub = node.advertise<PDG::FactList>("AGENT_MONITOR/factList", 1000);

    PDG::FactList factList_msg;
    PDG::Fact fact_msg;


    // Set this in a ros service?
    ros::Rate loop_rate(30);


    /************************/
    /* Start of the Ros loop*/
    /************************/

    while (node.ok()) {
        // We received agentMonitored

        Human* humCur;
        Robot* robCur;
        Joint* jntCur;

        //////////////////////////////////////
        //           Updating data          //
        //////////////////////////////////////


        if ((!humanMonitored && (robotRd.lastConfig_[agentMonitored] != NULL))
                || (humanMonitored && ( humanRd.lastConfig_[agentMonitored] != NULL) ) ) {
            // We add the agent to the mapTRBEntity and update roomOfInterest
            //printf("current human time: %lu\n", humanRd.lastConfig_[agentMonitored]->getTime());
            if (humanMonitored) {
                roomOfInterest = humanRd.lastConfig_[agentMonitored]->getRoomId();

                // We verify if the buffer is already there...
                itTRB = mapTRBEntity.find(agentMonitored);
                if (itTRB == mapTRBEntity.end()){

                    //1st time, we initialize variables
                    TRBuffer<Entity*> buffHum, buffJnt;

                    humCur = new Human(agentMonitored);
                    memcpy(humCur, humanRd.lastConfig_[agentMonitored], sizeof(Human));


                    buffHum.push_back(humCur->getTime(), humCur);
                    mapTRBEntity[humCur->getId()] = buffHum;                   
                    //printf("adding human named: reader %s, tmp %s, in buffer: %s\n", humanRd.lastConfig_[agentMonitored]->getName().c_str(), humCur->getName().c_str(), mapTRBEntity[humCur->getId()].back()->getName().c_str());

 
                    // adding monitored joint to the entities.
                    jntCur = new Joint(humCur->skeleton_[jointMonitoredName]->getId(), agentMonitored);
                    memcpy(jntCur, humanRd.lastConfig_[agentMonitored]->skeleton_[jointMonitoredName], sizeof(Joint));

                    buffJnt.push_back(jntCur->getTime(), jntCur);
 
                    mapTRBEntity[jntCur->getId()] = buffJnt;
                   // printf("adding joint named: reader %d %s, tmp %s, in buffer: %s\n", humCur->skeleton_[jointMonitoredName]->getId(), humCur->skeleton_[jointMonitoredName]->getName().c_str(), jntCur->getName().c_str(), mapTRBEntity[jntCur->getId()].back()->getName().c_str());
                    if(jointMonitoredId == 0)
                      jointMonitoredId = jntCur->getId();

                    ros::spinOnce();
                    loop_rate.sleep();
                    continue;

                // If this is a new data we add it to the buffer
                }else if((mapTRBEntity[agentMonitored].back()->getTime() < humanRd.lastConfig_[agentMonitored]->getTime())){

                    humCur = new Human(agentMonitored);
                    memcpy(humCur, humanRd.lastConfig_[agentMonitored], sizeof(Human));

                    mapTRBEntity[humCur->getId()].push_back(humCur->getTime(), humCur);
                    //printf("adding human named: reader %s, tmp %s, in buffer: %s\n", humanRd.lastConfig_[agentMonitored]->getName().c_str(), humCur->getName().c_str(), mapTRBEntity[humCur->getId()].back()->getName().c_str());

 
                    // adding monitored joint to the entities.
                    jntCur = new Joint(humCur->skeleton_[jointMonitoredName]->getId(), agentMonitored);
                    memcpy(jntCur, humanRd.lastConfig_[agentMonitored]->skeleton_[jointMonitoredName], sizeof(Joint));
 
                    mapTRBEntity[jntCur->getId()].push_back(jntCur->getTime(), jntCur);
                    //printf("adding joint named: reader %d %s, tmp %s, in buffer: %s\n", humCur->skeleton_[jointMonitoredName]->getId(), humCur->skeleton_[jointMonitoredName]->getName().c_str(), jntCur->getName().c_str(), mapTRBEntity[jntCur->getId()].back()->getName().c_str());
                    if(jointMonitoredId == 0)
                      jointMonitoredId = jntCur->getId();

                // Do we need this? Or should we update other entities?
                } else {
                    //printf("agent recieved without greater time: current is %lu < previous is %lu\n", humanRd.lastConfig_[agentMonitored]->getTime(), mapTRBEntity[agentMonitored].back()->getTime());

                    ros::spinOnce();
                    loop_rate.sleep();
                    continue;
                }
            // TODO: correct this as done for human
            } else {
                roomOfInterest = robotRd.lastConfig_[agentMonitored]->getRoomId();
                // If this is a new data we add it
                if ((mapTRBEntity[agentMonitored].empty()) || (mapTRBEntity[agentMonitored].back()->getTime() < robotRd.lastConfig_[agentMonitored]->getTime())) {
                    robCur = new Robot(agentMonitored);
                    memcpy(robCur, robotRd.lastConfig_[agentMonitored], sizeof(Robot));
                    mapTRBEntity[agentMonitored].push_back(robCur->getTime(), robCur);
                }
            }

            // for each entity
            //Put the following in a function?





            // For humans
            for (std::map<unsigned int, Human*>::iterator it = humanRd.lastConfig_.begin(); it != humanRd.lastConfig_.end(); ++it) {
                // if in same room as monitored agent and not monitored agent
                if (roomOfInterest == it->second->getRoomId() && it->first != agentMonitored) {
                  itTRB = mapTRBEntity.find(it->first);
                  if (itTRB == mapTRBEntity.end()){
                    TRBuffer<Entity*> buffHum;

                    Human * hum = new Human(it->first);
                    memcpy(hum, humanRd.lastConfig_[it->first], sizeof(Human));

                    buffHum.push_back(hum->getTime(), hum);
                    mapTRBEntity[it->first] = buffHum;
                    //printf("adding human name: reader %s, tmp %s, in buffer: %s\n", humanRd.lastConfig_[it->first]->getName().c_str(), hum->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str());
                  // If this is a new data we add it
                  }else if (mapTRBEntity[it->first].back()->getTime() < it->second->getTime()) {
                        Human * hum = new Human(it->first);
                        memcpy(hum, humanRd.lastConfig_[it->first], sizeof(Human));
                        mapTRBEntity[it->first].push_back(hum->getTime(), hum);
                        //printf("adding human name: reader %s, tmp %s, in buffer: %s\n", humanRd.lastConfig_[it->first]->getName().c_str(), hum->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str());
                    }
                } // TODO: else remove

            }






            // For robots

            for (std::map<unsigned int, Robot*>::iterator it = robotRd.lastConfig_.begin(); it != robotRd.lastConfig_.end(); ++it) {
                // if in same room as monitored agent and not monitored agent
                if ((roomOfInterest == it->second->getRoomId()) && (it->first != agentMonitored)) {
                  itTRB = mapTRBEntity.find(it->first);
                  if (itTRB == mapTRBEntity.end()){
                    TRBuffer<Entity*> buffRob;

                    Robot* rob  = new Robot(it->first);
                    memcpy(rob, robotRd.lastConfig_[it->first], sizeof(Robot));

                    buffRob.push_back(rob->getTime(), rob);
                    mapTRBEntity[it->first] = buffRob;
                    //printf("adding robot name: reader %s, tmp %s, in buffer: %s\n", robotRd.lastConfig_[it->first]->getName().c_str(), rob->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str());

                  // If this is a new data we add it
                  }else if( mapTRBEntity[it->first].back()->getTime() < it->second->getTime() ) {
                    Robot* rob  = new Robot(it->first);
                    memcpy(rob, robotRd.lastConfig_[it->first], sizeof(Robot));
                    mapTRBEntity[it->first].push_back(rob->getTime(), rob);
                    //printf("adding robot name: reader %s, tmp %s, in buffer: %s\n", robotRd.lastConfig_[it->first]->getName().c_str(), rob->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str());
                    }
                } // TODO: else remove

            }





            //  For Objects
            for (std::map<unsigned int, Object*>::iterator it = objectRd.lastConfig_.begin(); it != objectRd.lastConfig_.end(); ++it) {
                // if in same room as monitored agent and not monitored agent
                if ( roomOfInterest == it->second->getRoomId() ) {
                  itTRB = mapTRBEntity.find(it->first);
                  if (itTRB == mapTRBEntity.end()){
                    TRBuffer<Entity*> buffObj;

                    Object* obj  = new Object(it->first);
                    memcpy(obj, objectRd.lastConfig_[it->first], sizeof(Object));
                    
                    buffObj.push_back(obj->getTime(), obj);
                
                    mapTRBEntity[it->first] = buffObj;
                        //printf("adding object name: reader %s, tmp %s, in buffer: %s\n", objectRd.lastConfig_[it->first]->getName().c_str(), obj->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str());

                   // If this is a new data we add it
                   }else if( mapTRBEntity[it->first].back()->getTime() < it->second->getTime()) {
                        Object* obj  = new Object(it->first);
                        memcpy(obj, objectRd.lastConfig_[it->first], sizeof(Object));
                        mapTRBEntity[it->first].push_back(obj->getTime(), obj);
                        //printf("adding object name: reader %s, tmp %s, in buffer: %s\n", objectRd.lastConfig_[it->first]->getName().c_str(), obj->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str());
                    }
                } // TODO: else remove

            }




            //////////////////////////////////////////////
            // Compute facts concerning monitored agent //
            //////////////////////////////////////////////


            // Compute motion:
            unsigned long oneSecond = pow(10, 9);

            //if (!monitoredBufferInit) {
             //   printf("[AGENT_MONITOR][WARNING] agent monitored not found\n");
            //}else{
                //printf("[AGENT_MONITOR][DEBUG] agent from buffer %s is null? %d \n [AGENT_MONITOR][DEBUG] agent from reader %s is null? %d \n", mapTRBEntity[agentMonitored].back()->getName().c_str(),  mapTRBEntity[agentMonitored].back() == NULL, humanRd.lastConfig_[agentMonitored]->getName().c_str(), humanRd.lastConfig_[agentMonitored] == NULL);  
                //printf("[AGENT_MONITOR][WARNING] agent monitored buffer size %d, max_size %d, full %d, back is null? %d\n", mapTRBEntity[agentMonitored].size(), mapTRBEntity[agentMonitored].max_size(), mapTRBEntity[agentMonitored].full(), mapTRBEntity[agentMonitored].back() == NULL);
                if (computeMotion2D(mapTRBEntity[agentMonitored], oneSecond / 4, 0.03)) {
                    printf("[AGENT_MONITOR][DEBUG] %s is moving %lu\n", mapTRBEntity[agentMonitored].back()->getName().c_str(), mapTRBEntity[agentMonitored].back()->getTime());


                //Fact moving
                fact_msg.property = "isMoving";
                fact_msg.propertyType = "motion";
                fact_msg.subProperty = "agent";
                fact_msg.subjectId = 101;
                fact_msg.subjectName = mapTRBEntity[agentMonitored].back()->getName().c_str();
                fact_msg.stringValue = "true";
                fact_msg.confidence = 90;
                fact_msg.time = mapTRBEntity[agentMonitored].back()->getTime();

                    factList_msg.factList.push_back(fact_msg);


                    double angleDirection = 0.0;
                    std::map<unsigned int, double> mapIdValue;

                    // We compute the direction toward fact:
                    angleDirection = computeMotion2DDirection(mapTRBEntity[agentMonitored], oneSecond);
                    mapIdValue = computeMotion2DToward(mapTRBEntity, agentMonitored, angleDirection, 0.5);
                    for (std::map<unsigned int, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it) {
                        if (it->second > 0.0)
                            printf("[AGENT_MONITOR][DEBUG] %s is moving toward %s with a confidence of %f\n",
                                mapTRBEntity[agentMonitored].back()->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str(), it->second);

                    //Fact moving toward
                    fact_msg.property = "isMovingToward";
                    fact_msg.propertyType = "motion";
                    fact_msg.subProperty = "Direction";
                    fact_msg.subjectId = agentMonitored;
                    fact_msg.subjectName = mapTRBEntity[agentMonitored].back()->getName().c_str();
                    fact_msg.targetId = it->first;
                    fact_msg.targetName = mapTRBEntity[it->first].back()->getName().c_str();
                    fact_msg.confidence = it->second * 100;
                    fact_msg.time = mapTRBEntity[agentMonitored].back()->getTime();

                        factList_msg.factList.push_back(fact_msg);
                    }

                    // Then we compute /_\distance
                    mapIdValue = computeDeltaDist(mapTRBEntity, agentMonitored, oneSecond / 4);
                    for (std::map<unsigned int, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it) {
                        printf("[AGENT_MONITOR][DEBUG] agent %s has a deltadist toward  %s of %f\n",
                                mapTRBEntity[agentMonitored].back()->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str(), it->second);

                    //Fact moving toward
                    fact_msg.property = "isMovingToward";
                    fact_msg.propertyType = "motion";
                    fact_msg.subProperty = "Distance";
                    fact_msg.subjectId = agentMonitored;
                    fact_msg.subjectName = mapTRBEntity[agentMonitored].back()->getName().c_str();
                    fact_msg.targetId = it->first;
                    fact_msg.targetName = mapTRBEntity[it->first].back()->getName().c_str();
                    fact_msg.confidence = it->second * 100;
                    fact_msg.time = mapTRBEntity[agentMonitored].back()->getTime();

                    }

                // If agent is not moving, we compute his joint motion
                // TODO: do this in 3D!
                }else{
                   
                  std::map<unsigned int, double> mapIdValue;
                  double dist3D;
                  std::string dist3DString;

                  // What is the distance between joint and objects?
                  for (std::map<unsigned int, TRBuffer < Entity*> >::iterator it = mapTRBEntity.begin(); it != mapTRBEntity.end(); ++it) {
                    // if in same room as monitored agent and not monitored agent
                    if ((roomOfInterest == it->second.back()->getRoomId()) && (it->first != jointMonitoredId)) {
                      dist3D = bg::distance( mapTRBEntity[jointMonitoredId].back()->getPosition(), it->second.back()->getPosition() );
                                            
                      if( dist3D < 0.05 )
                        dist3DString = "reach";
                      else if( dist3D < 0.2 )
                        dist3DString = "close";
                      else if( dist3D < 1.5 )
                        dist3DString = "medium";
                      else if( dist3D < 8 )
                        dist3DString = "far";
                      else
                        dist3DString = "out";

                      //Fact distance
                      fact_msg.property = "distance";
                      fact_msg.propertyType = "position";
                      fact_msg.subProperty = "3D";
                      fact_msg.subjectId = jointMonitoredId;
                      fact_msg.subjectName = jointMonitoredName.c_str();
                      fact_msg.targetId = it->first;
                      fact_msg.targetName = mapTRBEntity[it->first].back()->getName().c_str();
                      fact_msg.valueType = 0;
                      fact_msg.stringValue = dist3DString;
                      fact_msg.doubleValue = dist3D;
                      fact_msg.confidence = 90;
                      fact_msg.time =  mapTRBEntity[jointMonitoredId].back()->getTime();

                      factList_msg.factList.push_back(fact_msg);

                    }
                  }
                  // Is the joint moving?
                  if (computeMotion2D(mapTRBEntity[jointMonitoredId], oneSecond / 4, 0.03)) {
                    printf("[AGENT_MONITOR][DEBUG] %s of agent %s is moving %lu\n", mapTRBEntity[jointMonitoredId].back()->getName().c_str(), mapTRBEntity[agentMonitored].back()->getName().c_str(), mapTRBEntity[agentMonitored].back()->getTime());


                    //Fact moving
                    fact_msg.property = "isMoving";
                        fact_msg.propertyType = "motion";
                        fact_msg.subProperty = "joint";
                    fact_msg.subjectId = jointMonitoredId;
                    fact_msg.subjectName = jointMonitoredName.c_str();
                    fact_msg.valueType = 0;
                    fact_msg.stringValue = "true";
                    fact_msg.confidence = 90;
                    fact_msg.time =  mapTRBEntity[jointMonitoredId].back()->getTime(); 

                    factList_msg.factList.push_back(fact_msg);


                    double angleDirection = 0.0;

                    // We compute the direction toward fact:
                    angleDirection = computeMotion2DDirection(mapTRBEntity[jointMonitoredId], oneSecond);
                    mapIdValue = computeMotion2DToward(mapTRBEntity, jointMonitoredId, angleDirection, 0.5);
                    for (std::map<unsigned int, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it) {
                        if (it->second > 0.0)
                            printf("[AGENT_MONITOR][DEBUG] %s of agent %s is moving toward %s with a confidence of %f\n", mapTRBEntity[jointMonitoredId].back()->getName().c_str(),
                                mapTRBEntity[agentMonitored].back()->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str(), it->second);

                        //Fact moving toward
                        fact_msg.property = "isMovingToward";
                            fact_msg.propertyType = "motion";
                        fact_msg.subProperty = "Direction";
                        fact_msg.subjectId = jointMonitoredId;
                        fact_msg.subjectName = jointMonitoredName.c_str();
                        fact_msg.targetId = it->first;
                        fact_msg.targetName = mapTRBEntity[it->first].back()->getName().c_str();
                        fact_msg.confidence = it->second * 100;
                        fact_msg.time = mapTRBEntity[jointMonitoredId].back()->getTime();

                        factList_msg.factList.push_back(fact_msg);
                    }

                    // Then we compute /_\distance
                    mapIdValue = computeDeltaDist(mapTRBEntity, jointMonitoredId, oneSecond / 4);
                    for (std::map<unsigned int, double>::iterator it = mapIdValue.begin(); it != mapIdValue.end(); ++it) {
                        printf("[AGENT_MONITOR][DEBUG] joint %s of agent %s has a deltadist toward  %s of %f\n", mapTRBEntity[jointMonitoredId].back()->getName().c_str(),
                                mapTRBEntity[agentMonitored].back()->getName().c_str(), mapTRBEntity[it->first].back()->getName().c_str(), it->second);

                        //Fact moving toward
                        fact_msg.property = "isMovingToward";
                        fact_msg.propertyType = "motion";
                        fact_msg.subProperty = "Distance";
                        fact_msg.subjectId = jointMonitoredId;
                        fact_msg.subjectName = mapTRBEntity[jointMonitoredId].back()->getName().c_str();
                        fact_msg.targetId = it->first;
                        fact_msg.targetName = mapTRBEntity[it->first].back()->getName().c_str();
                        fact_msg.confidence = it->second * 100;
                        fact_msg.time = mapTRBEntity[jointMonitoredId].back()->getTime();
                    }
                  }
                }
            //}
        }

        fact_pub.publish(factList_msg);

        ros::spinOnce();

        factList_msg.factList.clear();

        loop_rate.sleep();

    }
    return 0;
}
