#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>
#include <math.h>



RosThread::RosThread()
{
    shutdown = false;

    startMission = false;

    currentState = CS_STOP;

    qDebug()<<"currentCoalitionState: CS_STOP";

    // at the beginning of the mission, since each robot is singleton coalition,
    // all the robots are acting as a coalition leader.
    isCoalitionLeader = true;
}



void RosThread::work()
{

    if(!ros::ok()){

        emit this->rosFinished();

        return;
    }

    emit rosStarted();

    ros::Rate loop(10);


    QString path = QDir::homePath();
    path.append("/ISL_workspace/src/configISL.json");


    if(!readConfigFile(path)){

        qDebug()<< "Read Config File Failed!!!";

        ros::shutdown();

        emit rosFinished();
    }


    messageTaskInfo2CoordinatorPub = n.advertise<ISLH_msgs::taskInfo2CoordinatorMessage>("coalitionLeaderISLH/taskInfo2Coordinator",queueSize);

   // messageTaskInfo2CoordinatorDirect = n.advertise<messageDecoderISLH::taskInfo2CoordinatorMessage>("messageDecoder/taskInfoFromLeader",5);

    messageCmd2RobotsPub = n.advertise<ISLH_msgs::cmd2RobotsFromLeaderMessage>("coalitionLeaderISLH/cmd2Robots",queueSize);

    messageNewLeaderSub = n.subscribe("messageDecoderISLH/newLeader",queueSize,&RosThread::handleNewLeaderMessage, this);

    messageCmdFromCoordinatorSub = n.subscribe("messageDecoderISLH/cmdFromCoordinator",queueSize,&RosThread::handleCmdFromCoordinator,this);

    messageTaskInfoFromRobotSub = n.subscribe("messageDecoderISLH/taskInfoFromRobot",queueSize,&RosThread::handleTaskInfoMessage,this);

    coalInfo2MonitorPub = n.advertise<std_msgs::UInt8>("coalitionLeaderISLH/coalStateInfo2Monitor",queueSize);

    while(ros::ok())
    {

        if (isCoalitionLeader)
        {
            manageCoalition();
        }

        ros::spinOnce();

        loop.sleep();

    }

    qDebug()<<"I am quitting";

    ros::shutdown();

    emit rosFinished();


}
void RosThread::shutdownROS()
{
    ros::shutdown();
}

void RosThread::manageCoalition()
{
    if (currentState == CS_STOP)
    {
        if (startMission == true)
        {
            sendCmd2Robots(CMD_L2R_START_OR_STOP_MISSION);

            currentState = CS_IDLE;

            qDebug()<<"currentCoalitionState: CS_IDLE <- CS_STOP";

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }
    }
    // idle state
    else if (currentState == CS_IDLE)
    {
        if (waitingTasks.isEmpty()==false)
        {
            //check whether the coalition resource is sufficient for this task
            int sufficientResource = 1;
            for(int i = 0; i < coalTotalResources.size();i++)
            {
                if (coalTotalResources.at(i) < waitingTasks.at(0).requiredResources.at(i))
                {
                    sufficientResource = 0;
                    break;
                }
            }

            if (sufficientResource == 1)
            {
                //while checking resource excessiveness, the coalition leader may be splitted from
                // the coalition. Since the new coalition leader will be responsible for
                // coordinating the coalition, the waiting tasks' info will be sent to the new leader
                // If a new leader is determined, isCoalitionLeader is set to false, which means that
                // this robot (leader) is splitted from the coalition.
                check4ExcessiveResource();

                if (isCoalitionLeader==true)
                {
                    // check whether all the robot are in the task site

                    int taskSiteOK = 0;

                    double taskPoseX = waitingTasks.at(0).pose.X;
                    double taskPoseY = waitingTasks.at(0).pose.Y;

                    for(int robotIndx=0;robotIndx<coalMembers.size(); robotIndx++)
                    {
                        double robPoseX = coalMembers.at(robotIndx).pose.X;
                        double robPoseY = coalMembers.at(robotIndx).pose.Y;

                        double dist = sqrt((robPoseX-taskPoseX)*(robPoseX-taskPoseX) + (robPoseY-taskPoseY)*(robPoseY-taskPoseY));

                        if (dist <= targetSiteRadius*coalMembers.size())
                        {
                            taskSiteOK = taskSiteOK + 1;
                        }

                    }


                    if (taskSiteOK == coalMembers.size()) // all the robots in the coalition are in the task site
                    {
                        // consider the oldest task
                        handlingTask = waitingTasks.at(0);

                        // remove this task from waitingTasks
                        waitingTasks.remove(0);

                        // send start command to the coalition members
                        sendCmd2Robots(CMD_L2R_START_HANDLING);

                        handlingTask.startHandlingTime = QDateTime::currentDateTime().toTime_t();
                        handlingTask.status = 1;

                        // inform the coordinator of the starting of the handling
                        sendTaskInfo2Coordinator(INFO_L2C_START_HANDLING_WITH_TASK_INFO);

                        currentState = CS_HANDLING;

                        qDebug()<<"currentCoalitionState: CS_HANDLING <- CS_IDLE";


                        std_msgs::UInt8 coalInfoMsg;
                        coalInfoMsg.data = currentState;
                        coalInfo2MonitorPub.publish(coalInfoMsg);
                    }
                    else
                    {
                        sendTaskInfo2Coordinator(INFO_L2C_WAITING_TASK_SITE_POSE);

                        currentState = CS_WAITING_TASK_SITE_POSE;

                        qDebug()<<"currentCoalitionState: CS_WAITING_TASK_SITE_POSE  <- CS_IDLE";

                        std_msgs::UInt8 coalInfoMsg;
                        coalInfoMsg.data = currentState;
                        coalInfo2MonitorPub.publish(coalInfoMsg);
                    }
                }
            }
            else
            {
                sendTaskInfo2Coordinator(INFO_L2C_INSUFFICIENT_RESOURCE);

                currentState = CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR;

                qDebug()<<"currentCoalitionState: CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR <- CS_IDLE";

                std_msgs::UInt8 coalInfoMsg;
                coalInfoMsg.data = currentState;
                coalInfo2MonitorPub.publish(coalInfoMsg);
            }
        }
        else
        {
            // check whether all the robots reach the goal pose
            int goalPoseOK = 0;
            for(int robotIndx=0;robotIndx<coalMembers.size(); robotIndx++)
            {
                if (coalMembers.at(robotIndx).inGoalPose == 1)
                {
                    goalPoseOK = goalPoseOK + 1;
                }
            }

            // if all the coalition members are at their goal position
            // wait for new goal positions from the coordinator
            if (goalPoseOK == coalMembers.size())
            {
                for(int robotIndx=0;robotIndx<coalMembers.size(); robotIndx++)
                {
                    coalMembers[robotIndx].inGoalPose = 0;
                }

                sendTaskInfo2Coordinator(INFO_L2C_WAITING_GOAL_POSE);

                qDebug()<<"currentCoalitionState: CS_WAITING_GOAL_POSE <- "<<currentState;

                currentState = CS_WAITING_GOAL_POSE;                

                std_msgs::UInt8 coalInfoMsg;
                coalInfoMsg.data = currentState;
                coalInfo2MonitorPub.publish(coalInfoMsg);
            }

        }
    }
    // handling state
    else if (currentState == CS_HANDLING)
    {
        uint currentTime = QDateTime::currentDateTime().toTime_t();
        if (currentTime - handlingTask.startHandlingTime >= handlingTask.handlingDuration)
        {
            for(int robotIndx=0;robotIndx<coalMembers.size(); robotIndx++)
            {
                coalMembers[robotIndx].inGoalPose = 1;
                coalMembers[robotIndx].inTaskSite = 0;
            }

            sendTaskInfo2Coordinator(INFO_L2C_TASK_COMPLETED);

            completedTasks.push_back(handlingTask);

            currentState = CS_IDLE;

            qDebug()<<"currentCoalitionState: CS_IDLE <- CS_HANDLING";

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }
    }
    // waiting for a response from the coordinator
    else if (currentState == CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR)
    {
        uint currentTime = QDateTime::currentDateTime().toTime_t();
        if (currentTime - waitingTasks.at(0).encounteringTime >= waitingTasks.at(0).timeOutDuration)
        {            
            for(int robotIndx=0;robotIndx<coalMembers.size(); robotIndx++)
            {
                coalMembers[robotIndx].inGoalPose = 1;
                coalMembers[robotIndx].inTaskSite = 0;
            }
            timedoutTasks.push_back(waitingTasks.at(0));

            waitingTasks.remove(0);

            currentState = CS_IDLE;

            qDebug()<<"currentCoalitionState: CS_IDLE <- CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR";

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }
    }
    // succoring state
    else if (currentState == CS_SUCCORING)
    {
        // check whether all the robots reach the task site
        int taskSiteOK = 0;
        for(int robotIndx=0;robotIndx<coalMembers.size(); robotIndx++)
        {
            if (coalMembers.at(robotIndx).inTaskSite == 1)
            {
                taskSiteOK = taskSiteOK + 1;
            }
        }

        // if all the coalition members are in the task site
        if (taskSiteOK == coalMembers.size())
        {
            // consider the oldest task
            handlingTask = waitingTasks.at(0);

            // remove this task from waitingTasks
            waitingTasks.remove(0);

            // send start command to the coalition members
            sendCmd2Robots(CMD_L2R_START_HANDLING);

            handlingTask.startHandlingTime = QDateTime::currentDateTime().toTime_t();
            handlingTask.status = 1;

            // inform the coordinator of the starting of the handling
            sendTaskInfo2Coordinator(INFO_L2C_START_HANDLING);

            currentState = CS_HANDLING;

            qDebug()<<"currentCoalitionState: CS_HANDLING <- CS_SUCCORING";

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }
    }

}

// check whether the coalition has surplus of resources
// if yes, split as much as possible in order to maximize coalition value v(C_c,T)
void RosThread::check4ExcessiveResource()
{

    if (coalMembers.size()>1)
    {
        // find robots to be splitted such that the colition valu will increase by the splittings

        splitRobotIDList.clear();

        QVector <robotProp> coalMembersTmp1, coalMembersTmp2;//coalMmbrsTmp;

        coalMembersTmp1 = QVector <robotProp>(coalMembers);

        // current coalition value
        double coalVal = calcCoalValue(coalMembers);

        int leaderSplitted = 0;

        int changeAvail = 1;

        while(changeAvail==1)
        {
            changeAvail = 0;

            int splittedRobotIndx = -1;

            for(int robotIndx=0;robotIndx<coalMembersTmp1.size(); robotIndx++)
            {
                coalMembersTmp2 = QVector <robotProp>(coalMembersTmp1);
                coalMembersTmp2.remove(robotIndx);
                double coalValTemp = calcCoalValue(coalMembersTmp2);

                if (coalValTemp>=coalVal)
                {
                    if (coalMembersTmp1.at(robotIndx).robotID == ownRobotID)
                    {
                        leaderSplitted = 1;
                    }
                    // if the robot to be splitted is leader, ignore this splitting
                   // if (coalMembersTmp.at(robotIndx).robotID != ownRobotID)
                    //{
                        splittedRobotIndx = robotIndx;
                        coalVal = coalValTemp;
                        changeAvail = 1;

                        break;
                   // }
                }
            }

            if (splittedRobotIndx>-1)
            {
                // update the coalition resources due to the splitting
                for(int resID=0; resID<coalTotalResources.size(); resID++)
                {
                    coalTotalResources[resID] = coalTotalResources.at(resID) - coalMembersTmp1.at(splittedRobotIndx).resources.at(resID);
                }

                splitRobotIDList.append(coalMembersTmp1.at(splittedRobotIndx).robotID);

                // remove the splitted robot from the coalition
                coalMembersTmp1.remove(splittedRobotIndx);

            }
        }


        if (splitRobotIDList.size()>0)
        {

            //since the leader will ve removed from the coalition,
            //select new leader such that it has the smallest robot ID value
            newLeaderID = 9999;
            for(int i = 0; i < coalMembers.size();i++)
            {
                int robID = coalMembers.at(i).robotID;
                if ((robID != ownRobotID) && (robID < newLeaderID))
                {
                    bool flag = true;
                    for(int j = 0; j < splitRobotIDList.size();j++)
                    {
                        if (robID == splitRobotIDList.at(j))
                        {
                            flag = false;
                            break;
                        }
                    }

                    if (flag)
                        newLeaderID = robID;
                }
            }

            // inform the splitted robots of the splittng operation
            sendCmd2Robots(CMD_L2R_SPLIT_FROM_COALITION);

            //check whether one of the splitted robots is the coalition leader
            // if yes, select the robot with the smallest robotID as the coalition leader
            if (leaderSplitted==1)
            {
                sendCmd2Robots(CMD_L2R_LEADER_CHANGED);

                sendTaskInfo2Coordinator(INFO_L2C_SPLITTING_AND_LEADER_CHANGED);

                isCoalitionLeader = false;

            }
            else
            {
                // inform the coordinator of the splittings
                sendTaskInfo2Coordinator(INFO_L2C_SPLITTING);
            }

        }

    }

}

// calculate coalition's total resources
void  RosThread::calcCoalTotalResources(){
    coalTotalResources.clear();

    int numOfResources = coalMembers.at(0).resources.size();

    coalTotalResources.resize(numOfResources);

    for(int robID=0;robID<coalMembers.size();robID++){
        for(int resID=0;resID<numOfResources;resID++){
            coalTotalResources[resID] += coalMembers.at(robID).resources.at(resID);
        }
    }
}

// calculate the value of a given coaltion coalMmbrs
double RosThread::calcCoalValue(QVector <robotProp> coalMmbrs)
{

    double coalValue = 0.0;

    int numOfMem = coalMmbrs.size();

    int numOfResource = coalMmbrs.at(0).resources.size();

    double resDiff = 0;
    double excess = 0;

    for(int resID=0; resID<numOfResource; resID++)
    {
        double coalRes = 0.0;
        for(int robID=0; robID<numOfMem; robID++)
        {
            coalRes = coalRes + coalMmbrs.at(robID).resources.at(resID);
        }

        double taskRes = waitingTasks.at(0).requiredResources.at(resID);

        if ((taskRes-coalRes)>0)
        {
            resDiff = resDiff  + (taskRes-coalRes)*(taskRes-coalRes);
        }

        if (taskRes>0)
        {
            excess = excess + (1-coalRes/taskRes)*(1-coalRes/taskRes);
        }
    }


    double taskPoseX = waitingTasks.at(0).pose.X;
    double taskPoseY = waitingTasks.at(0).pose.Y;
    double dist = 0;
    for(int i=0;i<numOfMem; i++)
    {
        double robPoseX = coalMmbrs.at(i).pose.X;
        double robPoseY = coalMmbrs.at(i).pose.Y;

        dist = dist + sqrt((robPoseX-taskPoseX)*(robPoseX-taskPoseX) + (robPoseY-taskPoseY)*(robPoseY-taskPoseY))/(2*cvfParams.ro);
    }

    coalValue = 1.0/(1 + cvfParams.w1*resDiff +  cvfParams.w2*excess + cvfParams.w3*dist);

    return coalValue;
}

// prepare a command message which the leader sends to the coalition member(s)
void RosThread::sendCmd2Robots(int cmdType)
{
    ISLH_msgs::cmd2RobotsFromLeaderMessage msg;

    std::time_t sendingTime = std::time(0);
    msg.sendingTime = sendingTime;

    msg.cmdTypeID = cmdType;

    if (cmdType == CMD_L2R_START_OR_STOP_MISSION)
    {
        QString messageStr;
        if (startMission == true)
        {
            messageStr = "START-MISSION";
        }
        else
        {
            messageStr = "STOP-MISSION";
        }
        msg.cmdMessage = messageStr.toStdString();
        for(int i = 0; i < coalMembers.size();i++)
        {
            msg.receiverRobotID.push_back(coalMembers.at(i).robotID);
        }
    }
    else if (cmdType == CMD_L2R_START_HANDLING)
    {
        QString messageStr;
        messageStr.append(handlingTask.taskUUID);
        messageStr.append(",");
        messageStr.append(QString::number(handlingTask.handlingDuration));

        msg.cmdMessage = messageStr.toStdString();


        for(int i = 0; i < coalMembers.size();i++)
        {
            msg.receiverRobotID.push_back(coalMembers.at(i).robotID);
        }
    }
    else if ( (cmdType == CMD_L2R_MOVE_TO_TASK_SITE) || (cmdType == CMD_L2R_MOVE_TO_GOAL_POSE) )
    {
        qDebug()<<"sendCmd2Robots-> CMD_L2R_MOVE_TO_ " << robotTargetPosesStr;
        msg.cmdMessage = robotTargetPosesStr.toStdString();
        for(int i = 0; i < coalMembers.size();i++)
        {            
            msg.receiverRobotID.push_back(coalMembers.at(i).robotID);
        }

    }
    else if (cmdType == CMD_L2R_SPLIT_FROM_COALITION)
    {
        QString messageStr = "SPLIT";

        msg.cmdMessage = messageStr.toStdString();

        for(int i = 0; i < splitRobotIDList.size();i++)
        {
            msg.receiverRobotID.push_back(splitRobotIDList.at(i));
        }
    }
    else if (cmdType == CMD_L2R_LEADER_CHANGED)
    {

         QString messageStr = "NewLeaderID";
         messageStr.append(QString::number(newLeaderID));

         messageStr.append(":");

         // number of members in the new coalition
         messageStr.append(QString::number(coalMembers.size()-splitRobotIDList.size()));


         // send also coalition members' info to the new leader
         //robotID1;res1,res2,..,resn;posex,posey: ...
         for(int i = 0; i < coalMembers.size();i++)
         {
             int robID = coalMembers.at(i).robotID;
             if (robID != ownRobotID)
             {
                 for(int j = 0; j < splitRobotIDList.size();j++)
                 {
                     if (robID == splitRobotIDList.at(j))
                     {
                         robID = -1;
                         break;
                     }
                 }

                 if (robID>-1)
                 {
                     messageStr.append(":");

                     messageStr.append(QString::number(robID));
                     messageStr.append(";");
                     for(int j=0; j<coalMembers.at(i).resources.size();j++)
                     {
                        messageStr.append(QString::number(coalMembers.at(i).resources.at(j)));
                        if (j<coalMembers.at(i).resources.size()-1)
                            messageStr.append(",");
                     }
                     messageStr.append(";");
                     messageStr.append(QString::number(coalMembers.at(i).pose.X));
                     messageStr.append(",");
                     messageStr.append(QString::number(coalMembers.at(i).pose.Y));
                 }
             }
         }

         //send also the new leader the waiting tasks' info
         for(int wti = 0; wti < waitingTasks.size();wti++)
         {
             messageStr.append(":");

             messageStr.append(waitingTasks.at(wti).taskUUID);
             messageStr.append(";");
             messageStr.append(QString::number(waitingTasks.at(wti).pose.X));
             messageStr.append(";");
             messageStr.append(QString::number(waitingTasks.at(wti).pose.Y));
             messageStr.append(";");
             messageStr.append(QString::number(waitingTasks.at(wti).encounteringRobotID));
             messageStr.append(";");
             messageStr.append(QString::number(waitingTasks.at(wti).handlingDuration));
             messageStr.append(";");
             messageStr.append(QString::number(waitingTasks.at(wti).timeOutDuration));
             messageStr.append(";");
             messageStr.append(waitingTasks.at(wti).requiredResourcesString);
             messageStr.append(";");
             messageStr.append(QString::number(waitingTasks.at(wti).encounteringTime));
         }


        // the receivers of this message
        // the message's part after NewLeaderID is only sent to the new leader
        // this will be done by messageDecoderISLH

         msg.cmdMessage = messageStr.toStdString();
         for(int i = 0; i < coalMembers.size();i++)
         {
             int robID = coalMembers.at(i).robotID;
             for(int j = 0; j < splitRobotIDList.size();j++)
             {
                if (robID == splitRobotIDList.at(j))
                {
                    robID = -1;
                    break;
                }
             }
             if (robID>-1)
             {
                msg.receiverRobotID.push_back(robID);
             }
         }
    }
    else if (cmdType == CMD_L2R_I_AM_LEADER)
    {
        QString messageStr = "NewLeaderID";
        messageStr.append(QString::number(newLeaderID));

        msg.cmdMessage = messageStr.toStdString();

        for(int i = 0; i < coalMembers.size();i++)
        {
            int robID = coalMembers.at(i).robotID;

            msg.receiverRobotID.push_back(robID);
        }

    }
    else if (cmdType == CMD_L2R_NEW_ALL_TARGET_POSES)
    {
        msg.cmdMessage = targetPosesALLStr.toStdString();
        for(int i = 0; i < coalMembers.size();i++)
        {
            int robID = coalMembers.at(i).robotID;

            msg.receiverRobotID.push_back(robID);
        }
    }

    messageCmd2RobotsPub.publish(msg);
}


void RosThread::handleCmdFromCoordinator(ISLH_msgs::cmdFromCoordinatorMessage msg)
{

    qDebug()<< "handleCmdFromCoordinator-> messageTypeID: "<<msg.messageTypeID << "  message: "<< QString::fromStdString(msg.message);

    if (msg.messageTypeID == CMD_C2L_START_OR_STOP_MISSION)
    {
        QString msgStr = QString::fromStdString(msg.message);

        if (msgStr == "START-MISSION")
        {
            startMission = true;
        }
        else if (msgStr == "STOP-MISSION")
        {
            startMission = false;

            sendCmd2Robots(CMD_L2R_START_OR_STOP_MISSION);

            qDebug()<<"currentCoalitionState: CS_STOP <- "<<currentState;

            currentState = CS_STOP;

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }
    }
    else if (msg.messageTypeID == CMD_C2L_COALITION_MEMBERS)
    {
        // now this robot is the coalition leader
        isCoalitionLeader = true;

        // clear all the members
        coalMembers.clear();

        // robotTargetPosesStr = robotID1,posex1,posey1;robotID2,posex2,posey2;...;robotIDn,posexn,poseyn
        robotTargetPosesStr.clear();

        QString targetType = "0"; // targetType="g" -> move to goal; targetType="t" -> move to task site

        QString msgStr = QString::fromStdString(msg.message);

        // msgStr = numberOfMembers:robotID1;res1,res2,..,resn;posex,posey;goalOrTaskSite, targetx,targety:robotID2;res1,res2,...,resn;posex,posey;goalOrTaskSite,targetx,targety:task

        QStringList msgParts = msgStr.split(":",QString::SkipEmptyParts);

        int numOfMembers = msgParts.at(0).toInt();
        qDebug()<<"Number of coalition members"<<numOfMembers;

        // coalition members
        for(int i = 1; i <= numOfMembers;i++)
        {
            QStringList coalMemPropStr = msgParts.at(i).split(";",QString::SkipEmptyParts);

            robotProp robotTmp;

            robotTmp.robotID = coalMemPropStr.at(0).toInt();

            QStringList coalMemResStr = coalMemPropStr.at(1).split(",",QString::SkipEmptyParts);
            for(int j = 0; j < coalMemResStr.size();j++)
            {
               robotTmp.resources.append(coalMemResStr.at(j).toDouble());
            }

            QStringList coalMemPoseStr = coalMemPropStr.at(2).split(",",QString::SkipEmptyParts);
            robotTmp.pose.X = coalMemPoseStr.at(0).toDouble();
            robotTmp.pose.Y = coalMemPoseStr.at(1).toDouble();

            QStringList coalMemTargetStr = coalMemPropStr.at(3).split(",",QString::SkipEmptyParts);

            if (coalMemTargetStr.at(0)=="t")
            {
                robotTmp.taskSitePose.X = coalMemTargetStr.at(1).toDouble();
                robotTmp.taskSitePose.Y = coalMemTargetStr.at(2).toDouble();

                targetType = "t";
            }
            else if (coalMemTargetStr.at(0)=="g")
            {
                robotTmp.goalPose.X = coalMemTargetStr.at(1).toDouble();
                robotTmp.goalPose.Y = coalMemTargetStr.at(2).toDouble();

                targetType = "g";
            }

            robotTargetPosesStr.append(QString::number(robotTmp.robotID));
            robotTargetPosesStr.append(",");
            robotTargetPosesStr.append(coalMemTargetStr.at(1));
            robotTargetPosesStr.append(",");
            robotTargetPosesStr.append(coalMemTargetStr.at(2));
            if (i<numOfMembers)
                robotTargetPosesStr.append(";");


            // add this robot to coalMembers
            coalMembers.append(robotTmp);
        }

        // calculate new coalition total resources
        calcCoalTotalResources();

        newLeaderID = ownRobotID;

        // inform the members that I am the leader
        sendCmd2Robots(CMD_L2R_I_AM_LEADER);


        if (waitingTasks.size()>0)
            qDebug()<<"handleCmdFromCoordinator->CMD_C2L_COALITION_MEMBERS waitingTasks is not empty. taskUUID is "<<waitingTasks.at(0).taskUUID;

        waitingTasks.clear();

        if (msgParts.size()-1 > numOfMembers)
        {
            // proporties of the task to be handled

            QStringList taskMessageParts = msgParts.at(msgParts.size()-1).split(";",QString::SkipEmptyParts);
            taskProp taskTmp;

            taskTmp.taskUUID = taskMessageParts.at(0);
            taskTmp.pose.X = taskMessageParts.at(1).toInt();
            taskTmp.pose.Y = taskMessageParts.at(2).toInt();
            taskTmp.encounteringRobotID = taskMessageParts.at(3).toUInt();
            taskTmp.handlingDuration = taskMessageParts.at(4).toUInt();
            taskTmp.timeOutDuration = taskMessageParts.at(5).toUInt();
            taskTmp.requiredResourcesString = taskMessageParts.at(6);

            QStringList newTaskRRList = taskTmp.requiredResourcesString.split(",",QString::SkipEmptyParts);
            for(int i = 0; i < newTaskRRList.size();i++)
                taskTmp.requiredResources.append(newTaskRRList.at(i).toDouble());

            taskTmp.encounteringTime = taskMessageParts.at(7).toUInt();

            waitingTasks.append(taskTmp);
        }


        if (targetType == "g")
        {
            sendCmd2Robots(CMD_L2R_MOVE_TO_GOAL_POSE);

            qDebug()<<"currentCoalitionState: CS_IDLE <- "<<currentState;

            currentState = CS_IDLE;           

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }
        else if (targetType == "t")
        {

            sendCmd2Robots(CMD_L2R_MOVE_TO_TASK_SITE);

            qDebug()<<"currentCoalitionState: CS_SUCCORING <- "<<currentState;

            currentState = CS_SUCCORING;

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }

    }
    else if (msg.messageTypeID == CMD_C2L_LEADER_CHANGE)
    {
        QString msgStr = QString::fromStdString(msg.message);

        if(msgStr.toInt() == 1){
            isCoalitionLeader = false;
        }
    }
    else if ( (msg.messageTypeID == CMD_C2L_NEW_GOAL_POSES) || (msg.messageTypeID == CMD_C2L_NEW_TASK_SITE_POSES) )
    {

        QString msgStr = QString::fromStdString(msg.message);

        // msgStr = robotID1,posex1,posey1;robotID2,posex2,posey2;...;robotIDn,posexn,poseyn

        QStringList coalMmbrIDPoseList = msgStr.split(";",QString::SkipEmptyParts);

        for(int i = 0; i < coalMmbrIDPoseList.size();i++)
        {
            QStringList coalMmbrIDPoseStr = coalMmbrIDPoseList.at(i).split(",",QString::SkipEmptyParts);

            int rbtID =  coalMmbrIDPoseStr.at(0).toInt();

            for(int coalMbrIndx = 0; coalMbrIndx < coalMembers.size();coalMbrIndx++)
            {
                if (coalMembers.at(coalMbrIndx).robotID == rbtID)
                {
                    if (msg.messageTypeID == CMD_C2L_NEW_GOAL_POSES)
                    {
                        coalMembers[coalMbrIndx].goalPose.X = coalMmbrIDPoseStr.at(1).toDouble();
                        coalMembers[coalMbrIndx].goalPose.Y = coalMmbrIDPoseStr.at(2).toDouble();
                    }
                    else
                    {
                        coalMembers[coalMbrIndx].taskSitePose.X = coalMmbrIDPoseStr.at(1).toDouble();
                        coalMembers[coalMbrIndx].taskSitePose.Y = coalMmbrIDPoseStr.at(2).toDouble();
                    }
                    break;
                }
            }
        }

        robotTargetPosesStr = QString(msgStr);

        if (msg.messageTypeID == CMD_C2L_NEW_GOAL_POSES)
        {
            sendCmd2Robots(CMD_L2R_MOVE_TO_GOAL_POSE);

            qDebug()<<"currentCoalitionState: CS_IDLE <- "<<currentState;

            currentState = CS_IDLE;

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }
        else
        {
            sendCmd2Robots(CMD_L2R_MOVE_TO_TASK_SITE);

            qDebug()<<"currentCoalitionState: CS_SUCCORING <- "<<currentState;

            currentState = CS_SUCCORING;

            std_msgs::UInt8 coalInfoMsg;
            coalInfoMsg.data = currentState;
            coalInfo2MonitorPub.publish(coalInfoMsg);
        }
    }
    else if (msg.messageTypeID == CMD_C2L_NEW_ALL_TARGET_POSES)
    {
        targetPosesALLStr = QString::fromStdString(msg.message);

        sendCmd2Robots(CMD_L2R_NEW_ALL_TARGET_POSES);
    }

}

void RosThread::sendTaskInfo2Coordinator(int infoType)
{
    ISLH_msgs::taskInfo2CoordinatorMessage msg;

    std::time_t sendingTime = std::time(0);
    msg.sendingTime = sendingTime;

    msg.infoTypeID = infoType;
    msg.senderRobotID = ownRobotID;
    msg.receiverRobotID = coordinatorRobotID;

    if ( (infoType == INFO_L2C_INSUFFICIENT_RESOURCE) || (infoType == INFO_L2C_WAITING_TASK_SITE_POSE) )
    {
        msg.senderRobotID = ownRobotID;
        msg.taskUUID = waitingTasks.at(0).taskUUID.toStdString();
        msg.posX = waitingTasks.at(0).pose.X;
        msg.posY = waitingTasks.at(0).pose.Y;
        msg.encounteringTime = waitingTasks.at(0).encounteringTime;
        msg.taskResource = waitingTasks.at(0).requiredResourcesString.toStdString();
        msg.encounteringRobotID = waitingTasks.at(0).encounteringRobotID;
        msg.timeOutDuration = waitingTasks.at(0).timeOutDuration;
        msg.handlingDuration = waitingTasks.at(0).handlingDuration;
    }
    else if (infoType == INFO_L2C_START_HANDLING_WITH_TASK_INFO)
    {
        msg.senderRobotID = ownRobotID;
        msg.taskUUID = waitingTasks.at(0).taskUUID.toStdString();
        msg.posX = waitingTasks.at(0).pose.X;
        msg.posY = waitingTasks.at(0).pose.Y;
        msg.encounteringTime = waitingTasks.at(0).encounteringTime;
        msg.startHandlingTime = waitingTasks.at(0).startHandlingTime;
        msg.taskResource = waitingTasks.at(0).requiredResourcesString.toStdString();
        msg.encounteringRobotID = waitingTasks.at(0).encounteringRobotID;
        msg.timeOutDuration = waitingTasks.at(0).timeOutDuration;
        msg.handlingDuration = waitingTasks.at(0).handlingDuration;
    }
    else if (infoType == INFO_L2C_START_HANDLING)
    {
        msg.senderRobotID = ownRobotID;

        msg.startHandlingTime = handlingTask.startHandlingTime;

        msg.taskUUID = handlingTask.taskUUID.toStdString();
    }
   else if (infoType == INFO_L2C_TASK_COMPLETED)
    {
        msg.senderRobotID = ownRobotID;

        msg.taskUUID = handlingTask.taskUUID.toStdString();
    }
    else if ( (infoType == INFO_L2C_SPLITTING) || (infoType == INFO_L2C_SPLITTING_AND_LEADER_CHANGED) )
    {
        QString splittingMsg;

        for(int i = 0; i < splitRobotIDList.size();i++)
        {
            splittingMsg.append(QString::number(splitRobotIDList.at(i)));
            if (i<splitRobotIDList.size()-1)
                splittingMsg.append(",");
        }

        // if this robot, coalition leader, is splitted,
        // the splitted leader ID is added to the end of splitRobotIDList
        if (infoType == INFO_L2C_SPLITTING_AND_LEADER_CHANGED)
        {
            splittingMsg.append(",");
            splittingMsg.append(QString::number(newLeaderID));
        }

        msg.senderRobotID = ownRobotID;

        // this message contains the robot IDs to be splitted from the coalition
        msg.extraMsg = splittingMsg.toStdString();
    }
    else if (infoType == INFO_L2C_WAITING_GOAL_POSE)
    {
        QString messageStr = "GOALPOSE";
        msg.extraMsg = messageStr.toStdString();
    }


    messageTaskInfo2CoordinatorPub.publish(msg);
}

// Incoming task info message from a member robot
void RosThread::handleTaskInfoMessage(ISLH_msgs::taskInfoFromRobotMessage msg)
{
    if (msg.infoMessageType == INFO_R2L_NEW_TASK_INFO)
    {
        taskProp newTask;

        newTask.taskUUID = QString::fromStdString(msg.taskUUID);
        newTask.handlingDuration = msg.handlingDuration;
        newTask.timeOutDuration = msg.timeOutDuration;
        newTask.encounteringRobotID = msg.senderRobotID;

        QString newTaskRR =  QString::fromStdString(msg.requiredResources);
        qDebug()<< " Task - required resources " << newTaskRR;
        // Split the data (Comma seperated format)
        QStringList newTaskRRList = newTaskRR.split(",",QString::SkipEmptyParts);
        qDebug()<<"Number of resources parts"<<newTaskRRList.size();
        qDebug()<<newTaskRRList;
        for(int i = 0; i < newTaskRRList.size();i++)
        {
            newTask.requiredResources.append(newTaskRRList.at(i).toDouble());
        }

        newTask.requiredResourcesString = QString::fromStdString(msg.requiredResources);

        newTask.pose.X = msg.posX;
        newTask.pose.Y = msg.posY;

        newTask.startHandlingTime = -1;

        newTask.status = 0;

        newTask.encounteringTime = msg.encounteringTime;

        waitingTasks.append(newTask);
    }
    else if (msg.infoMessageType == INFO_R2L_REACHED_TO_TASK)
    {
        for(int i = 0; i < coalMembers.size();i++)
        {
            if (coalMembers.at(i).robotID == msg.senderRobotID)
            {
                coalMembers[i].inTaskSite = 1;
                break;
            }
        }
    }
    else if (msg.infoMessageType == INFO_R2L_REACHED_TO_GOAL)
    {
        for(int i = 0; i < coalMembers.size();i++)
        {
            if (coalMembers.at(i).robotID == msg.senderRobotID)
            {
                coalMembers[i].inGoalPose = 1;
                break;
            }
        }
    }

}

void RosThread::handleNewLeaderMessage(ISLH_msgs::newLeaderMessage msg)
{
    if (msg.infoTypeID == 1)
    {
        isCoalitionLeader = true;

        coalMembers.clear();

        QStringList messageParts = QString::fromStdString(msg.infoMessage).split(":",QString::SkipEmptyParts);
        int coalMemberCount = messageParts.at(1).toInt();
        int messagePartIdx = 2;
        for(; messagePartIdx < coalMemberCount + 2; messagePartIdx++){
            QStringList robotMessageParts = messageParts.at(messagePartIdx).split(";",QString::SkipEmptyParts);
            robotProp robot;

            robot.robotID = robotMessageParts.at(0).toUInt();

            QStringList resourceMessageParts = robotMessageParts.at(1).split(",",QString::SkipEmptyParts);
            for(int i=0;i<resourceMessageParts.size();i++)
                robot.resources.append(resourceMessageParts.at(i).toDouble());

            QStringList positionMessageParts = robotMessageParts.at(2).split(",",QString::SkipEmptyParts);
            robot.pose.X = positionMessageParts.at(0).toInt();
            robot.pose.Y = positionMessageParts.at(1).toInt();

            coalMembers.append(robot);
        }

        calcCoalTotalResources();

        waitingTasks.clear();

        for(; messagePartIdx < messageParts.size(); messagePartIdx++){
            QStringList taskMessageParts = messageParts.at(messagePartIdx).split(";",QString::SkipEmptyParts);
            taskProp task;

            task.taskUUID = taskMessageParts.at(0);
            task.pose.X = taskMessageParts.at(1).toInt();
            task.pose.Y = taskMessageParts.at(2).toInt();
            task.encounteringRobotID = taskMessageParts.at(3).toUInt();
            task.handlingDuration = taskMessageParts.at(4).toUInt();
            task.timeOutDuration = taskMessageParts.at(5).toUInt();
            task.requiredResourcesString = taskMessageParts.at(6);

            QStringList newTaskRRList = task.requiredResourcesString.split(",",QString::SkipEmptyParts);
            for(int i = 0; i < newTaskRRList.size();i++)
                task.requiredResources.append(newTaskRRList.at(i).toDouble());

            task.encounteringTime = taskMessageParts.at(7).toUInt();

            waitingTasks.append(task);
        }
    }
    else
    {
        // SPLITted from its coalition. Now it is a signleton coalition
        isCoalitionLeader = true;

        coalMembers.clear();
        waitingTasks.clear();

        robotProp robotTmp;

        robotTmp.robotID = ownRobotID;
        robotTmp.inGoalPose = 1;
        robotTmp.inTaskSite = -1;

        robotTmp.pose.X = -1;
        robotTmp.pose.Y = -1;

        robotTmp.goalPose.X = -1;
        robotTmp.goalPose.Y = -1;

        robotTmp.taskSitePose.X = -1;
        robotTmp.taskSitePose.Y = -1;

        robotTmp.radius = ownRobotRadius;

        robotTmp.resources = QVector <double>(ownRobotResources);

        coalMembers.append(robotTmp);

        calcCoalTotalResources();

    }
}

// Reads the config file
bool RosThread::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {
        cvfParams.w1 = result["cvf-w1"].toDouble();
        qDebug()<< " w1 " << cvfParams.w1;


        cvfParams.w2 = result["cvf-w2"].toDouble();
        qDebug()<< " w2 " << cvfParams.w2;

        cvfParams.w3 = result["cvf-w3"].toDouble();
        qDebug()<< " w3 " << cvfParams.w3;      

        cvfParams.ro = result["ro"].toDouble();
        qDebug()<< " ro " << cvfParams.ro;

        targetSiteRadius = result["targetSiteRadius"].toDouble();
        qDebug()<< " targetSiteRadius " << targetSiteRadius;


        ownRobotID = result["robotID"].toInt();
        qDebug()<< " ownRobotID " << ownRobotID;

        coordinatorRobotID = result["taskCoordinatorRobotID"].toInt();
        qDebug()<< " coordinatorRobotID " << coordinatorRobotID;

        queueSize = result["queueSize"].toInt();
        qDebug()<<result["queueSize"].toString();

        double radiusTmp = result["robotRadius"].toDouble();

        ownRobotRadius = radiusTmp;

        // initialize the singleton coalition

        QString resourceStr = result["resources"].toString();
        QStringList resourceStrList = resourceStr.split(",",QString::SkipEmptyParts);
        QVector <double> resources;
        for(int i = 0; i < resourceStrList.size();i++)
        {
           resources.append(resourceStrList.at(i).toDouble());
           ownRobotResources.append(resourceStrList.at(i).toDouble());
        }

        robotProp robotTmp;

        robotTmp.robotID = ownRobotID;
        robotTmp.inGoalPose = 1;
        robotTmp.inTaskSite = -1;

        robotTmp.pose.X = -1;
        robotTmp.pose.Y = -1;

        robotTmp.goalPose.X = -1;
        robotTmp.goalPose.Y = -1;

        robotTmp.taskSitePose.X = -1;
        robotTmp.taskSitePose.Y = -1;

        robotTmp.radius = radiusTmp;

        robotTmp.resources = resources;


        coalMembers.append(robotTmp);

        calcCoalTotalResources();


    }
    file.close();
    return true;

}
