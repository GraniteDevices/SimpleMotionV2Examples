#include "mw.h"
#include "ui_mw.h"
#include <QDateTime>
#include <math.h>

MW::MW(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MW)
{
    ui->setupUi(this);
    bushandle=-1;
    motionActive=busOpen=false;
    streamTime=0;

    ui->numOfAxis->setMaximum(maxAxis);

    //start timer which polls axis statuses
    timer=new QTimer(this);
    timer->setSingleShot(true);
    connect(timer,SIGNAL(timeout()),SLOT(timerTick()));
    timer->start(20);//first fire after 20ms

    updateUIcontrols();
}

MW::~MW()
{
    delete ui;
}

//set buttons enabled/disabled depending on state
void MW::updateUIcontrols()
{
    ui->connect->setEnabled(!busOpen);
    ui->disconnect->setEnabled(busOpen);
    ui->clearFaults->setEnabled(busOpen);
    if(busOpen)
    {
        ui->abortMotion->setEnabled(motionActive);
        ui->startMotion->setEnabled(!motionActive);
    }
    else
    {
        ui->abortMotion->setEnabled(false);
        ui->startMotion->setEnabled(false);
    }
}


void MW::on_connect_clicked()
{
    bushandle=smOpenBus(ui->portName->text().toLatin1());
    if(bushandle<0)
    {
        writeLog("Unable to open port, check that it's not used by another application and port name is correct");
        motionActive=busOpen=false;
    }
    else
    {
        writeLog("Port opened");
        busOpen=true;
        motionActive=false;
    }

    updateUIcontrols();
}

void MW::on_disconnect_clicked()
{
    smCloseBus(bushandle);
    bushandle=-1;
    motionActive=busOpen=false;
    writeLog("Port closed");

    updateUIcontrols();
}

void MW::on_startMotion_clicked()
{
    ui->settingsGroup->setEnabled(false);//lock settings

    motionActive=false;

    int i;
    for(i=0;i<ui->numOfAxis->value();i++)
    {
        //init axis. assume addresses be 1,2,3,...
        smBufferedInit(&axis[i],bushandle,i+1,ui->samplerate->currentText().toInt(),SMP_DEBUGPARAM5, SM_RETURN_VALUE_16B);

        if(logFiles.count()<=i)//open only if not yet opened
        {
            logFiles.append(new QFile(QString("C:/temp/SMBufferedLog_%1.txt").arg(i+1)));
            logFiles[i]->open(QIODevice::WriteOnly | QIODevice::Text);
            logStreams.append(new QTextStream(logFiles[i]));
        }
    }

    if(checkAndReportSMBusErrors())//if error occurred
    {
        on_abortMotion_clicked();//abort it
        updateUIcontrols();
        return;
    }

    motionActive=true;

    updateUIcontrols();
}

void MW::on_abortMotion_clicked()
{
    int i;
    for(i=0;i<ui->numOfAxis->value();i++)
    {
        //init axis. assume addresses be 1,2,3,...
        smBufferedAbort(&axis[i]);
        smBufferedDeinit(&axis[i]);

        logFiles[i]->close();
        //delete(logFiles[i]);
        //delete(logStreams[i]);
        //logStreams.clear();
        //logFiles.clear();
    }

    ui->settingsGroup->setEnabled(true);//unlock settings
    motionActive=false;

    updateUIcontrols();
}

void MW::on_clearFaults_clicked()
{
    int i;
    for(i=0;i<ui->numOfAxis->value();i++)
    {
        //setting faults register to 0 clears them if possible
        smSetParameter(bushandle,i+1,SMP_FAULTS,0);
    }
}

void MW::timerTick()
{
    //show timer counter in status bar to indicate that app is working
    static int tick=0;
    tick++;
    statusBar()->showMessage( QString("Update tick %1").arg(tick));

    feedDrives();

    //make timer to trigger again after 20ms
    timer->start(20);//next fire after 20ms
}

void MW::writeLog(QString msg)
{
    ui->log->append(QDateTime::currentDateTime().toString("hh:mm:ss")+ ": " +msg);
}


//return next point of motion stream
QList <double> MW::getNextTrajectoryCoordinates()
{
    QList <double> coords;

    //increment time
    streamTime+=1.0/ui->samplerate->currentText().toDouble();

    int i;

    //generate same sine setpoint for all axis
    for(i=0;i<maxAxis;i++)
    {
        double ampl=ui->setpointAmpl->value();
        double freq=ui->setpointFreq->value();
        coords.append(sin(2*M_PI*freq*streamTime)*ampl);
        //coords.append(sin(2*M_PI*freq*streamTime)*ampl*0.5+ampl);

        //coords.append(ampl);
    }

    return coords;
}


//return true if error happened
bool MW::checkAndReportSMBusErrors(smint32 smDeviceErrors)
{
    if(busOpen==false)
        return false;

    SM_STATUS smStat=getCumulativeStatus(bushandle);

    QString errorString;

    if( (smStat!=SM_OK && smStat!=SM_NONE) || (smDeviceErrors!=SMP_CMD_STATUS_ACK && smDeviceErrors!=0) )
    {
            QString errorFlags, smErrorFlags;
            //these faults are from SM bus host side
            if(smStat&SM_ERR_NODEVICE) errorFlags+="* NoDevice (interface)<br>";
            if(smStat&SM_ERR_PARAMETER) errorFlags+="* InvalidParameter (API)<br>";
            if(smStat&SM_ERR_COMMUNICATION) errorFlags+="* Communication (cheksum mismatch)<br>";
            if(smStat&SM_ERR_LENGTH) errorFlags+="* DataLegth (timeout or app error)<br>";
            if(smStat&SM_ERR_BUS) errorFlags+="* BusError<br>";

            //device errors are read from the device (so connection must be working). these are error flags of device side of SM bus
            if(smDeviceErrors&SMP_CMD_STATUS_NACK) smErrorFlags+="* Command fail (NACK)<br>";
            if(smDeviceErrors&SMP_CMD_STATUS_INVALID_ADDR) smErrorFlags+="* Invalid param address<br>";
            if(smDeviceErrors&SMP_CMD_STATUS_INVALID_VALUE) smErrorFlags+="* Invalid param value<br>";
            if(smDeviceErrors&SMP_CMD_STATUS_VALUE_TOO_HIGH) smErrorFlags+="* Value too high<br>";
            if(smDeviceErrors&SMP_CMD_STATUS_VALUE_TOO_LOW) smErrorFlags+="* Value too low<br>";

            errorString="";
            if(errorFlags.size())
                errorString+="Communication error flags: <br>"+errorFlags;
            else
                errorString="Communication error";
            if(smErrorFlags.size()) errorString+="<br>Device errors: <br>"+smErrorFlags;
    }

    resetCumulativeStatus(bushandle);

    if(!errorString.isEmpty())
    {
        writeLog(errorString);
        return true;
    }
    return false;
}

//call this as often as possible, preferrably in own thread in loop, but here for simplicity we call it from timer (may slow down GUI)
void MW::feedDrives()
{
    smint32 positions[maxAxis][64];//64 is a safe value, max amount of samples that we ever need to send at one cycle is 60
    smint32 readData[maxAxis][64];
    smint32 readDataAmount[maxAxis];
    int i,j;
    smint32 freeSpace[maxAxis];;

    //limiting amount of points buffered reduces latency of the buffer. however lowering buffer lenght also reduces tolarence to fill gaps, so filling must be more real time.
    int minimumBufferFreeBytes=axis[0].bufferLength - ui->maxBufferFillPercent->value()/100.0*axis[0].bufferLength;

    if(motionActive==false)
        return;

    //get amount of free space in first axis buffer. we do this only for first axis because rest of axes should have equal
    //or more free space. doing so saves unnecessary SM bus transmissions.
    smBufferedGetFree(&axis[0],&freeSpace[0]);
    /*for(i=0;i<ui->numOfAxis->value();i++)
    {
        smBufferedGetFree(&axis[i],&freeSpace[i]);

        if(axis[i].bufferFill==0)
            writeLog(QString("Empty buffer detected on %1 (the first fill, or buffer underrun)").arg(i));
    }*/

    //update buffer fill bar in UI
    ui->bufferFill->setValue(axis[0].bufferFill);

    //if error occurs, abort motion. TODO think some more graceful way to stop maybe.
    if(checkAndReportSMBusErrors()==true)
    {
        on_abortMotion_clicked();
        return;
    }

    while(smBufferedGetMaxFillSize(&axis[0],freeSpace[0])>1  && freeSpace[0] >= minimumBufferFreeBytes )
    {
        //get info of how much points we are allowed to send to devices in one fill
        int maxpoints=smBufferedGetMaxFillSize(&axis[0],freeSpace[0]);

        //generate coordinate stream
        for(j=0;j<maxpoints;j++)
        {
            QList <double> coords=getNextTrajectoryCoordinates();

            for(i=0;i<ui->numOfAxis->value();i++)
            {
                positions[i][j]=smint32(coords[i]);
            }
        }

        //send to devices, and receive read data back
        for(i=0;i<ui->numOfAxis->value();i++)
        {
            smint32 bytesFilled;
            
            /*In the next call new setpoints are sent to a single drive and returned data points are stored in the
            readData 2D table. Number of data points returned are stored in readDataAmount.

            For more information of data flow in this function, see http://granitedevices.com/wiki/Buffered_motion_stream_in_SimpleMotion_V2
            
            bytesFilled will indicate how many bytes were actually written to device buffer
            */
            smBufferedFillAndReceive(&axis[i],maxpoints,positions[i],&readDataAmount[i],readData[i],&bytesFilled);
            
            //write feedback to log files
            for(int a=0;a<readDataAmount[i];a++)
            {
                smuint32 d=readData[i][a];
  //              int vset=char(d&0xff);
//                int vfb=char((d&0xff00)>>8);
                int vset=char(d<<1);
                int vfb=char(char((d)>>7)<<1);
                smuint8 tim=0;
                //smuint8 tim=(d&0xff0000)>>16;
                //(*logStreams[i])<<QString("%2\n").arg(readData[i][a]);
                (*logStreams[i])<<QString("%1 %2 %3\n").arg(tim).arg(vset).arg(vfb);
            }
            
            //reduce buffer free counter by the amount we just consumed in the fill            
            freeSpace[0]-=bytesFilled;
        }
    }

    //synchronize clocks of all devices to the current value of first axis
    smBufferedRunAndSyncClocks(&axis[0]);
}


void MW::on_readFaults_clicked()
{
    int i;
    for(i=0;i<ui->numOfAxis->value();i++)
    {
        smint32 faultbits,faultloc,debug1;
        smRead3Parameters(bushandle,i+1,SMP_DEBUGPARAM6,&faultbits,SMP_DEBUGPARAM2,&faultloc,SMP_DEBUGPARAM1,&debug1);
        writeLog(QString("Axis %1: faultbits %2, location %3, debug1 %4").arg(i).arg(faultbits,0,10).arg(faultloc).arg(debug1));
    }
}
