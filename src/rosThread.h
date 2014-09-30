#include <ros/ros.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>
#include <QtCore/QString>
#include <ISLH_msgs/taskInfo2CoordinatorMessage.h>
#include <ISLH_msgs/newTaskInfoMessage.h>
#include <ISLH_msgs/cmdFromCoordinatorMessage.h>
#include <ISLH_msgs/cmd2RobotsFromLeaderMessage.h>
#include <ISLH_msgs/taskInfoFromRobotMessage.h>
#include <ISLH_msgs/newLeaderMessage.h>
#include <std_msgs/UInt8.h>

enum CoalitionStatus
{
    CS_STOP = -1,
    CS_IDLE = 0,
    CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR = 1,
    CS_SUCCORING = 2,
    CS_HANDLING = 3,
    CS_WAITING_GOAL_POSE = 4,
    CS_WAITING_TASK_SITE_POSE = 5
};

enum Robot2LeaderInfoMgs
{
    INFO_R2L_NEW_TASK_INFO = 1,
    INFO_R2L_REACHED_TO_TASK = 2,
    INFO_R2L_REACHED_TO_GOAL = 3
};

enum Leader2RobotCmdMsgs
{
    CMD_L2R_START_OR_STOP_MISSION = 0,
    CMD_L2R_START_HANDLING = 1,
    CMD_L2R_MOVE_TO_TASK_SITE = 2,
    CMD_L2R_MOVE_TO_GOAL_POSE = 3,
    CMD_L2R_SPLIT_FROM_COALITION = 4,
    CMD_L2R_LEADER_CHANGED = 5,
    CMD_L2R_NEW_ALL_TARGET_POSES = 6,
    CMD_L2R_I_AM_LEADER = 7
};

enum Leader2CoordinatorInfoMgs
{
    INFO_L2C_INSUFFICIENT_RESOURCE = 1,
    INFO_L2C_START_HANDLING = 2,
    INFO_L2C_START_HANDLING_WITH_TASK_INFO = 3,
    INFO_L2C_TASK_COMPLETED = 4,
    INFO_L2C_SPLITTING = 5,
    INFO_L2C_SPLITTING_AND_LEADER_CHANGED = 6,
    INFO_L2C_WAITING_GOAL_POSE = 7,
    INFO_L2C_WAITING_TASK_SITE_POSE = 8
};


enum Coordinator2LeaderCmdMsgs
{
    CMD_C2L_START_OR_STOP_MISSION = 0,
    CMD_C2L_COALITION_MEMBERS = 1,
    CMD_C2L_LEADER_CHANGE = 2,
    CMD_C2L_NEW_GOAL_POSES = 3,
    CMD_C2L_NEW_TASK_SITE_POSES = 4,
    CMD_C2L_NEW_ALL_TARGET_POSES = 5
};

struct coalValFuncParams{
    double w1;
    double w2;
    double w3;
    double ro;
};

struct poseXY{
    double X;
    double Y;
};

struct robotProp{
    uint robotID;
    QVector <double> resources;
    double radius;
    poseXY pose;    
    poseXY goalPose;
    poseXY taskSitePose;
    int inTaskSite;
    int inGoalPose;
};


// task properties
struct taskProp{
  QString taskUUID;
  uint encounteringTime; // in timestamp - "the time when the task is encountered"
  uint responsibleUnit;  // "who is responsible for the task"
  uint encounteringRobotID;  // "Id of the robot encountering the task"
  uint handlingDuration; // in seconds - "the required time to handle the task"
  uint timeOutDuration; // "the timed-out duration for the task"
  int status; // "status of the task"
              //
  uint startHandlingTime; // in timestamp - "the time when the task starts being handled"
  poseXY pose; // the location of the task
  QVector < double > requiredResources;
  QString requiredResourcesString;
};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();


private:
     bool shutdown;

     ros::NodeHandle n;

     ros::Publisher messageTaskInfo2CoordinatorPub;

     // If the task coordinator is also a coalition leader,
     // get the message directly from coalitionLeaderISLH
     ros::Publisher messageTaskInfo2CoordinatorDirectPub;

     // send the command to the robot(s)
     ros::Publisher messageCmd2RobotsPub;

     ros::Subscriber messageNewLeaderSub;

     ros::Subscriber messageCmdFromCoordinatorSub;

     ros::Subscriber messageTaskInfoFromRobotSub;

     ros::Publisher coalInfo2MonitorPub;

     bool isCoalitionLeader;

     bool startMission;

     int ownRobotID;

     int coordinatorRobotID;

     int newLeaderID;

     int queueSize;

     double targetSiteRadius;

     QString targetPosesALLStr;

     //QVector <taskProp> newTasksList;

     QVector <taskProp> waitingTasks;
     QVector <taskProp> completedTasks;
     QVector <taskProp> timedoutTasks;

     taskProp handlingTask;

     QVector <double> coalTotalResources;

     QVector <robotProp> coalMembers;

     QVector <int> splitRobotIDList;

     // used to send the incoming goal or task site pose from the coordinator to the member robots
     // after sending, it is cleared
     //QVector <robotTargetPosesProp> robotTargetPoses;
     QString robotTargetPosesStr;

     coalValFuncParams cvfParams; // the parameters w1, w2, w3, adn ro in the coalition value function

     CoalitionStatus currentState;

     QVector <poseXY> goalPoses;

     void manageCoalition();

     void check4ExcessiveResource();

     double calcCoalValue(QVector <robotProp> coalMmbrs);

     void sendCmd2Robots(int cmdType);

     void sendTaskInfo2Coordinator(int infoType);

     void handleTaskInfoMessage(ISLH_msgs::taskInfoFromRobotMessage msg);

     void handleCmdFromCoordinator(ISLH_msgs::cmdFromCoordinatorMessage msg);

     void handleNewLeaderMessage(ISLH_msgs::newLeaderMessage msg);

     bool readConfigFile(QString filename);

     void calcCoalTotalResources();

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void rosStarted();
   void rosStartFailed();

};
