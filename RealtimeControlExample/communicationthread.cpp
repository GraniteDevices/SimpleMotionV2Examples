#include "communicationthread.h"
#include <math.h>

CommunicationThread::CommunicationThread(QObject *parent): QThread(parent)
{
    targetAddress=1;
    useHighBaudRate=false;
    updateFrequency=50;
    trackingErrorLimit=12000;
    proportionalGain=10;
    positionSetpoint=0;
    maxVelocitySetpoint=50;
    trackingErrorFault=false;
    running=false;
    clearDriveErrorsFlag=false;
    prevDriveFaultStopState=false;
    prevServoRearyState=false;
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
    posSetpointIncrement=change;
    tasks.append(Task::IncrementSetpoint);
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
            trackingErrorFault=false;
            positionSetpoint=positionFeedback;
            emit logMessage("Tracking error cleared, position target have been set as current feedback position and velocity setpoint has been released");
            break;
        case Task::IncrementSetpoint:
            positionSetpoint+=posSetpointIncrement;
            posSetpointIncrement=0;
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
        smSetBaudrate(460800);//set SM default baudrate which is needed to reconnect after increased baudrate (so needed for consequent connect if "use high baudrate" option was set)
        busHandle=smOpenBus(portName.toLatin1());
        if(busHandle<0)
        {
            emit logMessage("Opening SM bus failed");
            return;
        }

        resetCumulativeStatus( busHandle );//reset possible SM errors

        //read some variables from device
        smint32 controlMode, deviceType, actualPosition, driveStatus, capabilities1, FWversion, smProtocolVersion,encoderResolutionPPR, maxBPS;
        smRead3Parameters(busHandle,targetAddress,SMP_SM_VERSION,&smProtocolVersion,SMP_DEVICE_TYPE,&deviceType, SMP_BUS_SPEED|SMP_MAX_VALUE_MASK, &maxBPS);

        //check if above reads failed in any way
        if(checkAndReportSMBusErrors())
        {
            emit logMessage("Start aborted.");
            smCloseBus(busHandle);
            return;
        }

        //detect ARGON drive
        if((deviceType/1000)==4)
        {
            emit logMessage(QString("ARGON drive detected, it might not be supported in this application at the moment. However, if you get 'Drive compatibility checked & passed' message in the log, then it's going to work."));
        }


        //required SM protocol version is 28 or greater to guarantee next commands to run sucessfully (especially SMP_DEVICE_CAPABILITIES1)
        if(smProtocolVersion<28)
        {
            emit logMessage(QString("Target drive connected, but it has too old SimpleMotion protocol support (version %1). Try upgrading drive firmware.").arg(smProtocolVersion));
            emit logMessage("Start aborted.");
            smCloseBus(busHandle);
            return;
        }

        smRead3Parameters(busHandle,targetAddress,SMP_CONTROL_MODE,&controlMode,SMP_ACTUAL_POSITION_FB_NEVER_RESETTING,&actualPosition,SMP_ENCODER_PPR,&encoderResolutionPPR);
        smRead3Parameters(busHandle,targetAddress,SMP_STATUS,&driveStatus, SMP_DEVICE_CAPABILITIES1,&capabilities1,SMP_FIRMWARE_VERSION,&FWversion);

        //check if above reads failed in any way
        if(checkAndReportSMBusErrors())
        {
            emit logMessage("Start aborted.");
            smCloseBus(busHandle);
            return;
        }

        positionFeedback=positionSetpoint=actualPosition;
        feedbackDeviceResolution=encoderResolutionPPR*4;

        //stringify control mode
        QString CM="Unknown";
        if(controlMode==CM_NONE) CM="None";
        else if(controlMode==CM_POSITION) CM="Position";
        else if(controlMode==CM_VELOCITY) CM="Velocity";
        else if(controlMode==CM_TORQUE) CM="Torque";

        emit logMessage(QString("Connected to device with type ID %1 with firmware version %4, which is in %2 control mode and has initial position feedback at %3 encoder counts, and encoder resolution is %5. Drive's max supported bus baud rate is %6 BPS.").arg(deviceType).arg(CM).arg(actualPosition).arg(FWversion).arg(feedbackDeviceResolution).arg(maxBPS));

        //check drive state & configuration:
        bool abort=false;

        //check if drive is in right mode
        if(controlMode!=CM_VELOCITY)
        {
            emit logMessage("This demo app works only if drive is in Velocity control mode, which it's not. To use this app, configure drive to be in Velocity control mode with Granity.");
            abort=true;
        }

        if(!(driveStatus&STAT_SERVO_READY))
        {
            emit logMessage("Drive does not seem to be ready for control (status register bit 'Ready for use' is False). Configure & enable drive with Granity before using this app.");
            abort=true;
        }

        if(!(driveStatus&STAT_SERVO_READY))
        {
            emit logMessage("Drive does not seem to be ready for control (status register bit 'Ready for use' is False). Configure & enable drive with Granity before using this app.");
            abort=true;
        }

        if(!(capabilities1&DEVICE_CAPABILITY1_SELECTABLE_FAST_UPDATE_CYCLE_FORMAT))
        {
            emit logMessage("Connected drive firmware does not support required DEVICE_CAPABILITY1_SELECTABLE_FAST_UPDATE_CYCLE_FORMAT. Try drive upgrading firmware to latest version.");
            abort=true;
        }

        if(!(capabilities1&DEVICE_CAPABILITY1_CONTROL_BITS1_VERSION2))
        {
            emit logMessage("Connected drive firmware does not support required DEVICE_CAPABILITY1_CONTROL_BITS1_VERSION2. Try drive upgrading firmware to latest version.");
            abort=true;
        }

        //if some error above occurred
        if(abort)
        {
            emit logMessage("Start aborted.");
            smCloseBus(busHandle);
            return;
        }

        smSetDebugOutput(High,stderr);
        //change bitrate
        if(useHighBaudRate)
        {
            //maxBPS=1000000; //uncomment to test with other than max baudrate

            //first set device timeout (watchdog/fault behavior), so if connection is lost, they reset to default baud rate after a certain time period
            //note: we change these settings of all bus devices simultaneously because errors will happen if not all devices have same BPS (address 0=broadcast to all)
            const int deviceTimeoutMs=150;//max valid value 10230 ms. however, this should be set _less_ than timeout period of SM host (see SM_READ_TIMEOUT). timeout of SM host can be changed before smOpenBus if needed.
            smSetParameter(busHandle,0,SMP_FAULT_BEHAVIOR,(deviceTimeoutMs/10)<<8);//set timeout
            smSetParameter(busHandle,0,SMP_BUS_SPEED,maxBPS);//set baudrate

            //if all went ok, now device is in new baud rate, switch host PBS too
            smCloseBus(busHandle);
            smSetBaudrate(maxBPS);
            busHandle=smOpenBus(portName.toLatin1());
            if(busHandle<0)
            {
                emit logMessage(QString("Opening SM bus failed with high baud rate (%1 BPS), perhaps bus device doesn't support high BPS").arg(maxBPS));
                emit logMessage("Start aborted.");
                return;
            }

            //test that new speed works
            resetCumulativeStatus( busHandle );//reset possible SM errors
            smint32 devType2;
            smRead1Parameter(busHandle,targetAddress,SMP_DEVICE_TYPE,&devType2);//just reading some parameter to test

            //check if above SM commands failed in any way
            if(checkAndReportSMBusErrors() || devType2!=deviceType ) //we also compare that value is same than before but it's quite unnecessary as automatic CRC check will catch any data error that might happen
            {
                emit logMessage("Device didn't respond corrently at new baudrate, perhaps RS485 bus termination is missing and causing errors at high speed?");
                smCloseBus(busHandle);
                emit logMessage("Start aborted.");
                return;
            }
        }


        //change smFastUpdateCycle data format
        smSetParameter(busHandle,targetAddress,SMP_FAST_UPDATE_CYCLE_FORMAT,FAST_UPDATE_CYCLE_FORMAT_ALT1);


        //check if above SM commands failed in any way
        if(checkAndReportSMBusErrors())
        {
            emit logMessage("Start aborted.");
            smCloseBus(busHandle);
            return;
        }

        running=true;
        trackingErrorFault=false;
        emit logMessage("Drive compatibility checked & passed. Control started.");
    }

    running=true;
}

void CommunicationThread::DoStopAndDisconnect()
{
    if(running)
    {
        //stop motion
        smSetParameter(busHandle,targetAddress,SMP_ABSOLUTE_SETPOINT,0);
        smCloseBus(busHandle);
        running=false;
        emit logMessage("Stopped");
    }
}

void CommunicationThread::DoSetParams()
{
    emit logMessage("Parameters set");
}

void CommunicationThread::DoResetDriveErrors()
{
    clearDriveErrorsFlag=true;
}


//if fast=true then do only checks that do not need communication via SM bus (local checks only,
//such as errors in received packets, but not reporting errors in invalid parameter values)*/
bool CommunicationThread::checkAndReportSMBusErrors(bool fast)
{
    QString errs;

    /*SM bus & SM devices have three categories of status & error bits:
     *
     *1) Bus status & error bits. These are returned on each SM library call (the SM_STAUTS type)
     *   and accumulated into a internal variable that may be read by getCumulativeStatus function.
     *   This value reports errors that happen on with communication layer (phyiscal device problems
     *   such as not available of bus device or checksum error).
     *
     *2) Device side SM status & error bits. reading this requires working connection to a target device
     *   in order to read SMP_CUMULATIVE_STATUS parameter.
     *   This value contains errors that successfully were transferred to target but were not accepted
     *   by some reason (i.e. if invalid paramter address or value was used).
     *
     *3) Device specific state & errors, such as servo drive stauts and fault bits on SMP_STATUS and
     *   SMP_FAULTS parameters. These states are not checked in this function.
     *
     */


    //read SMP_CUMULATIVE_STATUS
    smint32 SMDeviceSideCommStatus;
    if(fast==false)
    {
        smRead1Parameter(busHandle,targetAddress,SMP_CUMULATIVE_STATUS,&SMDeviceSideCommStatus);
        //if we have some error bits on, reset them, so we can spot new errors later
        if(SMDeviceSideCommStatus!=0)
        {
            smSetParameter(busHandle,targetAddress,SMP_CUMULATIVE_STATUS,0);
        }
    }
    else
    {
        SMDeviceSideCommStatus=0;//a cludge to avoid false errors being reported in stringifySMBusErrors
    }

    //read cumulative bus status errors and all convert (1) and (2) bits to human readable form:
    errs=stringifySMBusErrors(getCumulativeStatus(busHandle), SMDeviceSideCommStatus);

    //reset local errors bits
    resetCumulativeStatus(busHandle);

    //if there were errors, log them
    if(errs.isEmpty()==false)
    {
       emit logMessage(errs);
       return true;
    }

    return false;
}

QString CommunicationThread::stringifySMBusErrors(SM_STATUS smStat, smint32 smDeviceErrors)
{
    QString errorString;

    if( ((smStat!=SM_OK && smStat!=SM_NONE) || smDeviceErrors!=SMP_CMD_STATUS_ACK ))
    {
            QString errorFlags, smErrorFlags;
            //these faults are from SM bus host side
            if(smStat&SM_ERR_NODEVICE) errorFlags+="* NoDevice (bus is not opened)<br>";
            if(smStat&SM_ERR_PARAMETER) errorFlags+="* InvalidParameter (invalid access to a target device parameter, i.e. read/write unsupported parameter address, or writing value that is not allowed to a parameter)<br>";
            if(smStat&SM_ERR_COMMUNICATION) errorFlags+="* Communication (received data cheksum mismatch)<br>";
            if(smStat&SM_ERR_LENGTH) errorFlags+="* DataLegth (not enough return data received from device)<br>";
            if(smStat&SM_ERR_BUS) errorFlags+="* BusError (communication port device error)<br>";

            if(!(smStat&SM_ERR_NODEVICE))//ignore device side faults if nodevice is active because it would make no sense
            {
                //device errors are read from the device (so connection must be working). these are error flags of device side of SM bus
                if(smDeviceErrors&SMP_CMD_STATUS_NACK) smErrorFlags+="* Command fail (NACK)<br>";
                if(smDeviceErrors&SMP_CMD_STATUS_INVALID_ADDR) smErrorFlags+="* Invalid param address<br>";
                if(smDeviceErrors&SMP_CMD_STATUS_INVALID_VALUE) smErrorFlags+="* Invalid param value<br>";
                if(smDeviceErrors&SMP_CMD_STATUS_VALUE_TOO_HIGH) smErrorFlags+="* Value too high<br>";
                if(smDeviceErrors&SMP_CMD_STATUS_VALUE_TOO_LOW) smErrorFlags+="* Value too low<br>";

            }
            errorString="";
            if(errorFlags.size())
                errorString+="Bus error flags: <br>"+errorFlags+"<br>";
            else
                errorString="Communication error.";
            if(smErrorFlags.size()) errorString+="<br>Device errors: <br>"+smErrorFlags;
    }

    return errorString;
}

//this is the core function that does the actual control and uses smFastUpdateCycle to transmit setpoint & motor feedback and drive status/control bits
void CommunicationThread::DoUpdateCycle()
{
    int  velocitySetpoint;

    /* this app uses fast update cycle format 1 (ALT1):
    *
    *  description: this type has 28 bits absolute setpoint and 30 bits absolute feedback value + 4 output bits for control + 2 input bits for status
    *  write1=lowest 16 bits absolute setpoint value
    *  write2=bits 0-11: upper 12 bits of absolute setpoint value, bits 12-15: these bits are written as bits 0-3 of SMP_CONTROL_BITS_1. See SMP_CB1_nn for functions. So write2 bit 15=bypass trajplanner 14=set quickstop 13=clear faults 12=enable.
    *  read1=lowest 16 bits of position feedback value
    *  read2=bit nr 16 = STAT_SERVO_READY, bit nr 15=STAT_FAULTSTOP, bits 0-14=upper bits of position feedback value (pos FB bits 17-30)
    */

    //calculate new velocity setpoint by using proportional error amplifier from position tracking error
    int posTrackingError=positionSetpoint-positionFeedback;

    if(abs(posTrackingError)>trackingErrorLimit )
    {
        if(trackingErrorFault==false)
            emit logMessage(QString("Application entered in position tracking error state (position tracking error was %1). This means that velocity setpoint has been forced to 0. Try clearing faults to resume.").arg(posTrackingError));

        trackingErrorFault=true;
    }

    if(trackingErrorFault)
        velocitySetpoint=0;
    else
        velocitySetpoint=round(1000.0*(double)posTrackingError*proportionalGain/feedbackDeviceResolution);//note: we sacle output with inverse of feedbackDeviceResolution to have somewhat same gain sensitivity regardless of motor sensor

    smuint32 writeLong=((smuint32)velocitySetpoint)&0x0fffffff;

    if(clearDriveErrorsFlag)
    {
        writeLong|=0x20000000;//write clearfaults flag with fast command
        clearDriveErrorsFlag=false;
        emit logMessage("Drive errors cleared");
    }

    writeLong|=0x10000000;//write enable with fast command. without this, drive gets disabled
    writeLong|=0x80000000;//write bypass trajectory planner with fast command

    smuint16 write1=0,write2=0,read1,read2;

    //split long into 16 bit short values
    write1=writeLong&0xffff;
    write2=writeLong>>16;

    //send the fast update cycle to drive & check errors
    if(smFastUpdateCycle(busHandle,targetAddress,write1,write2,&read1,&read2)!=SM_OK)
    {
        //report & handle error
        checkAndReportSMBusErrors();
        stopAndDisconnect();
    }

    //extract data from read1 & read2
    positionFeedback=int(read1 | (smuint32(read2&0x3fff)<<16));
    positionFeedback=(positionFeedback<<2)>>2;//use bit shift trick to extend sign of pos feedback (fill the 2 missing MSB bits with sign of 30 bit value). this might not work with all compilers but at least with gcc it does.
    bool faultstop=read2&0x4000;//get fault stop state of drive
    bool servoready=read2&0x8000;//get servo ready state of drive

    //drive just faulted, report it once
    if(faultstop==true &&  prevDriveFaultStopState==false)
    {
        emit logMessage("Drive entered in fault state. Try clear faults to resume.");
    }
    prevDriveFaultStopState=faultstop;

    //warn about servo ready==false but better handling of it is not implemented here
    if(servoready==false && prevServoRearyState==true)
    {
        emit logMessage("Drive 'Ready for use' flag is became false (drive not enabled?). Control will not work until it's true.");
    }
    prevServoRearyState=servoready;

    emit updateReadings(positionSetpoint,positionFeedback,velocitySetpoint);
}
