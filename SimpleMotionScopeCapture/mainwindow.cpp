#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <unistd.h> //for usleep
#include "simplemotion.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //enable low amount of debug output to report SM bus errors to stderr
    smSetDebugOutput(SMDebugMid,stderr);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_capture_clicked()
{
    nodeAddress = ui->deviceAddress->value(); //SM device address
    int samples[2048];

    ui->log->appendPlainText("Opening port");
    handle = smOpenBus(ui->busName->text().toLatin1());

    if (handle < 0)
    {
        ui->log->appendPlainText("Could not open bus");
        return;
    }

    if(setupScope()==false)
    {
        ui->log->appendPlainText("Setting scope parameters failed, probaly device didn't respond to SimpleMotion commands.");
        smCloseBus(handle);
        return;
    }

    ui->log->appendPlainText("Scope is active, waiting for ready state");
    if(scopeWait()!=SM_OK)
    {
        ui->log->appendPlainText("scopeWait indicated SM communication error");
        smCloseBus(handle);
        return;
    }

    ui->log->appendPlainText("Downloading");
    qApp->processEvents();//update GUI manually because this app is not multithreaded and blocks UI thread in processing
    if(downloadScope( 2048, samples )==false)
    {
        ui->log->appendPlainText("downloadScope indicated SM communication error");
        smCloseBus(handle);
        return;
    }

    //sample list will be repeating list of samples defined to SMP_CAPTURE_SOURCE.
    //I.e if we capture 3 channels A, B and C, then samples will contain A0,B0,C0,A1,B1,C1,A2,B3,C3 etc
    for( int i=0; i<2048; i++)
    {
        ui->log->appendPlainText(QString("samples[%1]=%2").arg(i).arg(samples[i]) );
    }

    ui->log->appendPlainText("All done");
    smCloseBus(handle);
    return;
}

bool MainWindow::setupScope()
{
    SM_STATUS status = 0;

    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_BUF_LENGHT, 2048);//SM device has scope buffer length of at least 2048 samples
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_BEFORE_TRIGGER_PERCENTS, 0); //note, requires on IONI FW 1.6.0 or later
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_SAMPLERATE, 7); //with IONI sample rate is 20000/(SMP_CAPTURE_SAMPLERATE+1) Hz, so here it is 2500 Hz
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_SOURCE, BV(CAPTURE_POSITION_ACTUAL)|BV(CAPTURE_BUS_VOLTAGE));//this defines which channels to capture
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_TRIGGER, TRIG_INSTANT);//specify scope trigger
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_STATE, 1); //start waiting for trigger (if TRIG_INSTANT used, then it's immediate)

    if(status!=SM_OK)
    {
        //error
        return false;
    }
    return true;
}

//parameters: nsamples=number of samples to read 1-2048, samples=pointer to buffer with at least nsamples length
bool MainWindow::downloadScope( int nsamples, int *samples )
{
    smint32 retval;
    int samplesread=0, storepos=0;
    SM_STATUS smStat=0;

    //setup for scope reading
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_SET_PARAM_ADDR, SMP_RETURN_PARAM_LEN );
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_24B, SMPRET_32B );//read 32 bit (values capped to 30 bits) samples
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_SET_PARAM_ADDR, SMP_RETURN_PARAM_ADDR );
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_24B, SMP_CAPTURE_BUFFER_GET_VALUE ); //read at get_value param
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_SET_PARAM_ADDR, SMP_CAPTURE_BUFFER_GET_ADDR ); //set store address to get_addr
    smStat|=smExecuteCommandQueue(handle,nodeAddress);
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );

    //loop to read samples
    while(samplesread<nsamples)
    {
        int i;
        int samplestofetch=nsamples-samplesread;
        if(samplestofetch>30)samplestofetch=30;//maximum per one SM cycle (4*30=120 bytes=max payload)

        //add read param commands to queue
        for(i=0;i<samplestofetch;i++)
        {
            smStat|=smAppendSMCommandToQueue( handle, SMPCMD_24B, samplesread );
            samplesread++;
        }

        //transmit & redeive over SM bus
        smStat|=smExecuteCommandQueue(handle,nodeAddress);

        //read values from return data queue
        for(i=0;i<samplestofetch;i++)
        {
            smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
            samples[storepos++]=retval;
        }
    }

    //read one dummy variable just to cause SMP_RETURN_PARAM_ADDR to change to non-SMP_CAPTURE_BUFFER_GET_VALUE so next time we read data, we get it all from beginning
    smStat |= smRead1Parameter(handle, nodeAddress, SMP_NULL, &retval );

    if(smStat!=SM_OK)
        return false;

    return true;
}

SM_STATUS MainWindow::scopeWait()
{
    smint32 state;

    //loop until SMP_CAPTURE_STATE is 0 (scope idle)
    while (true)
    {
        SM_STATUS status = smRead1Parameter(handle, nodeAddress, SMP_CAPTURE_STATE, &state);

        if (status != SM_OK)
            return status;

        ui->log->appendPlainText(QString("Scope state: %1").arg(state));

        if (state == 0)
        {
            break;
        }

        qApp->processEvents();//update GUI manually because this app is not multithreaded and blocks UI thread in processing
        usleep(100000);
    }

    return SM_OK;
}
