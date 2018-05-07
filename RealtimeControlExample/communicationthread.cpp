#include "communicationthread.h"

CommunicationThread::CommunicationThread(QObject *parent): QThread(parent)
{
    targetAddress=1;
    useHighBaudRate=false;
    updateFrequency=50;
    trackingErrorLimit=2000;
    proportionalGain=10;
    positionSetpoint=0;
    maxVelocitySetpoint=50;
    trackingErrorFault=false;
    running=false;
    busHandle=-1;
}

CommunicationThread::~CommunicationThread()
{
    qDebug()<<"destroy comm";
    mutex.lock();
    tasks.append(Task::Quit);
    waitCondition.wakeOne();
    mutex.unlock();
    if(wait(5000)==false)
    {
        //called when app quits but thread wont close (never should happen)
        qDebug()<<"terminating thread";
        setTerminationEnabled();
        terminate();
    }
}


void CommunicationThread::setPortDetails(QString portname, int targetAddress, bool useHighBaudRate)
{
    QMutexLocker locker(&mutex);//this locks mutex and fres it automatically when we leave this function

    this->portName=portname;
    this->targetAddress=targetAddress;
    this->useHighBaudRate=useHighBaudRate;
}

void CommunicationThread::connectAndStart()
{
    QMutexLocker locker(&mutex);//this locks mutex and fres it automatically when we leave this function
    tasks.append(Task::ConnectAndStart);
}

void CommunicationThread::stopAndDisconnect()
{
    QMutexLocker locker(&mutex);//this locks mutex and fres it automatically when we leave this function
    tasks.append(Task::StopAndDisconnect);
}

void CommunicationThread::setParameters(int updateFrequency, int trackingErrorLimit, double proportionalGain, double maxVelocitySetpoint)
{
    QMutexLocker locker(&mutex);//this locks mutex and fres it automatically when we leave this function

    this->updateFrequency=updateFrequency;
    this->trackingErrorLimit=trackingErrorLimit;
    this->proportionalGain=proportionalGain;
    this->maxVelocitySetpoint=maxVelocitySetpoint;

    tasks.append(Task::SetParams);
}

void CommunicationThread::clearTrackingError()
{
    QMutexLocker locker(&mutex);//this locks mutex and fres it automatically when we leave this function
    tasks.append(Task::ClearTrackingError);
}

void CommunicationThread::clearDriveErrors()
{
    QMutexLocker locker(&mutex);//this locks mutex and fres it automatically when we leave this function
    tasks.append(Task::ClearDriveErrors);
}

void CommunicationThread::incrementSetpoint(int change)
{
    QMutexLocker locker(&setpointLock);//this locks mutex and fres it automatically when we leave this function
    positionSetpoint+=change;
}

void CommunicationThread::quitApp()
{
    QMutexLocker locker(&mutex);//this locks mutex and fres it automatically when we leave this function
    tasks.append(Task::Quit);
}

void CommunicationThread::run()
{
    Task currentTask;

    while(true)
    {
        mutex.lock();
        if(tasks.isEmpty()==false)//we have new task pending
        {
            currentTask=tasks.takeFirst();
        }
        else
            currentTask=Task::None;
        mutex.unlock();

        //execute task
        switch(currentTask)
        {
        case Task::ConnectAndStart:
            DoConnectAndStart();
            break;
        case Task::StopAndDisconnect:
            DoStopAndDisconnect();
            break;
        case Task::SetParams:
            DoSetParams();
            break;
        case Task::ClearDriveErrors:
            DoResetDriveErrors();
            break;
        case Task::ClearTrackingError:
            emit logMessage("Drive errors cleared");
            trackingErrorFault=false;
            break;
        case Task::Quit:
            DoStopAndDisconnect();
            return;
            break;
        case Task::None:
        default:
            break;
        }

        //do servo control
        if(running)
        {
            DoUpdateCycle();
        }

        mutex.lock();
        waitCondition.wait(&mutex,1000.0/double(updateFrequency));
        mutex.unlock();
    }
}

void CommunicationThread::DoConnectAndStart()
{
    if(running==false)
    {
        busHandle=smOpenBus(portName.toLatin1());
        if(busHandle<0)
        {
            emit logMessage("Opening SM bus failed");
            return;
        }

        //todo check its in vel mode
        //change bitrate

        running=true;
        trackingErrorFault=false;
        emit logMessage("Started");
    }

    running=true;
}

void CommunicationThread::DoStopAndDisconnect()
{
    if(running)
    {
        smuint16 read1, read2;

        //stop motion
        smFastUpdateCycle(busHandle,targetAddress,0,0,&read1,&read2);
        smCloseBus(busHandle);
        running=false;
        emit logMessage("Stopped");
    }
}


void CommunicationThread::DoUpdateCycle()
{
    int posSetpoint, feedbackPos, veolictySetpoint;
    setpointLock.lock();
    posSetpoint=positionSetpoint;
    setpointLock.unlock();

    emit updateReadings(posSetpoint,feedbackPos,veolictySetpoint);
}

void CommunicationThread::DoSetParams()
{
    emit logMessage("Parameters set");
}

void CommunicationThread::DoResetDriveErrors()
{
    emit logMessage("Drive errors cleared");

}

