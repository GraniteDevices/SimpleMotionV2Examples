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
    void setParameters(int updateFrequency, int trackingErrorLimit, double proportionalGain );
    void clearTrackingError();
    void clearDriveErrors();
    void incrementSetpoint(int change);
    void quitApp();

signals:
    void logMessage(QString text);
    void updateReadings(int setpoint, int feedbackpos, int velocitysetpoint);
    void errorDetected(bool trackingError, bool driveFault);
    void runningAndConnectedStateChanged(bool running);

protected:
    void run();

private:
    void DoStopAndDisconnect();
    void DoConnectAndStart();    
    void DoSetParams();
    void DoResetDriveErrors();
    //this is the core function that does the actual control and uses smFastUpdateCycle to transmit setpoint & motor feedback and drive status/control bits:
    void DoUpdateCycle();

    QString stringifySMBusErrors(SM_STATUS smStat, smint32 smDeviceErrors);
    bool checkAndReportSMBusErrors(bool fast=false);

    enum class Task { None, ConnectAndStart, StopAndDisconnect, SetParams, ClearTrackingError, ClearDriveErrors, IncrementSetpoint, Quit };
    QMutex mutex, setpointLock;
    QList <Task> tasks;
    QWaitCondition waitCondition;

    QString portName;
    int targetAddress;
    bool useHighBaudRate;
    int updateFrequency;
    int trackingErrorLimit;
    double proportionalGain;
    int positionSetpoint;
    int posSetpointIncrement;
    int positionFeedback;
    double feedbackDeviceResolution;
    bool trackingErrorFault;
    bool running;
    bool clearDriveErrorsFlag;
    bool prevDriveFaultStopState;
    bool prevServoRearyState;
    smbus busHandle;
};

#endif // COMMUNICATIONTHREAD_H
