
// hlk byrm @ 2014

#include <ros/ros.h>
#include <QApplication>
#include <QThread>
#include "rosThread.h"


int main(int argc,char** argv){

    QApplication app(argc,argv);



    ros::init(argc,argv,"coalitionLeaderISLH");

    RosThread* rosthread = new RosThread;

    QThread* worker = new QThread(&app);

    rosthread->moveToThread(worker);


    QObject::connect(rosthread,SIGNAL(rosFinished()),worker,SLOT(quit()));
    QObject::connect(worker,SIGNAL(finished()),&app,SLOT(quit()));

    QObject::connect(worker,SIGNAL(finished()),rosthread,SLOT(deleteLater()));

    QObject::connect(worker,SIGNAL(started()),rosthread,SLOT(work()));


    worker->start();


    return app.exec();
}
