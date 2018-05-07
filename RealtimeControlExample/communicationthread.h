#ifndef COMMUNICATIONTHREAD_H
#define COMMUNICATIONTHREAD_H

#include <QObject>
#include <QtCore>
#include "simplemotion.h"

class CommunicationThread: public QThread
{
    Q_OBJECT

public:
    CommunicationThread(QObject *parent);
    ~CommunicationThread();

    //interface from UI
    void setPortDetails(QString portname, int targetAddress, bool useHighBaudRate );
    void connectAndStart();
    void stopAndDisconnect();
    void setParameters(int updateFrequency, int trackingErrorLimit, double proportionalGain, double maxVelocitySetpoint );
    void clearTrackingError();
    void clearDriveErrors();
    void incrementSetpoint(int change);
    void quitApp();

signals:
    void logMessage(QString text);
    void updateReadings(int setpoint, int feedbackpos, int velocitysetpoint);

protected:
    void run();

private:
    void DoStopAndDisconnect();
    void DoConnectAndStart();
    void DoUpdateCycle();
    void DoSetParams();
    void DoResetDriveErrors();


    enum class Task { None, ConnectAndStart, StopAndDisconnect, SetParams, ClearTrackingError, ClearDriveErrors, Quit };
    QMutex mutex, setpointLock;
    QList <Task> tasks;
    QWaitCondition waitCondition;

    QString portName;
    int targetAddress;
    bool useHighBaudRate;
    int updateFrequency;
    int trackingErrorLimit;
    double proportionalGain;
    double maxVelocitySetpoint;
    int positionSetpoint;
    bool trackingErrorFault;
    bool running;
    smbus busHandle;
};

#endif // COMMUNICATIONTHREAD_H
