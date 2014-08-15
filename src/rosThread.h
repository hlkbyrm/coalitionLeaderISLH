#include <ros/ros.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>
#include <QtCore/QString>
#include <coalitionLeaderISLH/taskInfo2CoordinatorMessage.h>
#include <taskObserverISLH/newTaskInfoMessage.h>
#include <messageDecoderISLH/cmdFromCoordinatorMessage.h>
//#include <messageDecoderISLH/cmd2RobotsMessage.h>
#include <coalitionLeaderISLH/cmd2RobotsFromLeaderMessage.h>
#include <messageDecoderISLH/taskInfoFromRobotMessage.h>
#include <messageDecoderISLH/newLeaderMessage.h>

enum CoalitionState
{
    CS_IDLE = 0,
    CS_WAITING_TASK_RESPONSE_FROM_COORDINATOR = 1,
    CS_SUCCORING = 2,
    CS_HANDLING = 3
};

enum Leader2RobotCmdMsgs
{
    CMD_L2R_START_HANDLING = 1,
    CMD_L2R_MOVE_TO_TASK_SITE = 2,
    CMD_L2R_MOVE_TO_GOAL_POSE = 3,
    CMD_L2R_SPLIT_FROM_COALITION = 4,
    CMD_L2R_LEADER_CHANGED = 5
};

enum Leader2CoordinatorInfoMgs
{
    INFO_L2C_INSUFFICIENT_RESOURCE = 1,
    INFO_L2C_START_HANDLING = 2,
    INFO_L2C_TASK_COMPLETED = 3,
    INFO_L2C_SPLITTING = 4,
    INFO_L2C_LEADER_CHANGED = 5
};


enum Coordinator2LeaderCmdMsgs
{
    CMD_C2L_COALITION_MEMBERS = 1
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
    poseXY pose;
    bool inTaskSite;
    bool inGoalPose;
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

     bool isCoalitionLeader;

     int ownRobotID;

     int coordinatorRobotID;

     //QVector <taskProp> newTasksList;

     QVector <taskProp> waitingTasks;
     QVector <taskProp> completedTasks;
     QVector <taskProp> timedoutTasks;

     taskProp handlingTask;

     QVector <double> coalTotalResources;

     QVector <robotProp> coalMembers;

     QVector <int> splitRobotIDList;

     coalValFuncParams cvfParams; // the parameters w1, w2, w3, adn ro in the coalition value function

     CoalitionState currentState;

     QVector <poseXY> goalPoses;

     void manageCoalition();

     void check4ExcessiveResource();

     double calcCoalValue(QVector <robotProp> coalMmbrs);

     void sendCmd2Robots(int cmdType);

     void sendTaskInfo2Coordinator(int infoType);

     void handleTaskInfoMessage(messageDecoderISLH::taskInfoFromRobotMessage msg);

     void handleCmdFromCoordinator(messageDecoderISLH::cmdFromCoordinatorMessage msg);

     void handleNewLeaderMessage(messageDecoderISLH::newLeaderMessage msg);

     bool readConfigFile(QString filename);

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void rosStarted();
   void rosStartFailed();

};
