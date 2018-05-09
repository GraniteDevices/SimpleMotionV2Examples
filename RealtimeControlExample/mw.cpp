#include "mw.h"
#include "ui_mw.h"

MW::MW(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MW),
    commThread(parent)
{
    ui->setupUi(this);
    connect(&commThread,SIGNAL(logMessage(QString)),this,SLOT(logMessage(QString)));
    connect(&commThread,SIGNAL(updateReadings(int,int,int)),this,SLOT(updateStatus(int,int,int)));
    connect(&commThread,SIGNAL(errorDetected(bool,bool)),this,SLOT(handleErrorDetected(bool,bool)));
    connect(&commThread,SIGNAL(runningAndConnectedStateChanged(bool)),this,SLOT(runningAndConnectedStateChanged(bool)));
    commThread.start();
    runningAndConnectedStateChanged(false);
    logMessage("App version 1.0.0 ready");
}

MW::~MW()
{
    delete ui;
}

void MW::on_connect_clicked()
{
    commThread.setPortDetails(ui->busName->text(),ui->deviceAddress->value(),ui->useHighBaudRate->isChecked());
    commThread.connectAndStart();
}

void MW::on_disconnect_clicked()
{
    commThread.stopAndDisconnect();
}

void MW::on_decrement10000_clicked()
{
    commThread.incrementSetpoint(-10000);
}

void MW::on_decrement1000_clicked()
{
    commThread.incrementSetpoint(-1000);
}

void MW::on_decrement100_clicked()
{
    commThread.incrementSetpoint(-100);
}

void MW::on_decrement10_clicked()
{
    commThread.incrementSetpoint(-10);
}

void MW::on_increment10_clicked()
{
    commThread.incrementSetpoint(10);
}

void MW::on_increment100_clicked()
{
    commThread.incrementSetpoint(100);
}

void MW::on_increment1000_clicked()
{
    commThread.incrementSetpoint(1000);
}

void MW::on_increment10000_clicked()
{
    commThread.incrementSetpoint(10000);
}

void MW::on_resetLocalTrackingError_clicked()
{
    commThread.clearTrackingError();
    ui->resetLocalTrackingError->setStyleSheet("");
}

void MW::on_resetDriveFaults_clicked()
{
    commThread.clearDriveErrors();
    ui->resetDriveFaults->setStyleSheet("");
}

void MW::on_updateRate_valueChanged(int arg1)
{
    applySettigns();
}

void MW::on_positionGain_valueChanged(double arg1)
{
    applySettigns();
}

void MW::on_trackingErrorTolerance_valueChanged(int arg1)
{
    applySettigns();
}

void MW::logMessage(QString text)
{
    ui->log->append("<p>"+text+"</p>");
}

void MW::updateStatus(int posSetpoint, int posFeedback, int velSetpoint)
{
    ui->posSetpoint->display(posSetpoint);
    ui->velocitySetpoint->display(velSetpoint);
    ui->measuredPosition->display(posFeedback);
}

void MW::handleErrorDetected(bool trackingError, bool driveFault)
{
    if(trackingError)
        ui->resetLocalTrackingError->setStyleSheet("color: rgb(0, 0, 0);background-color: rgb(255, 170, 0);");
    if(driveFault)
        ui->resetDriveFaults->setStyleSheet("color: rgb(0, 0, 0);background-color: rgb(255, 170, 0);");
}

void MW::runningAndConnectedStateChanged(bool running)
{
    //update UI controls enabled/disabled state depending on connection status
    ui->connect->setEnabled(!running);
    ui->disconnect->setEnabled(running);
    ui->busName->setEnabled(!running);
    ui->deviceAddress->setEnabled(!running);
    ui->useHighBaudRate->setEnabled(!running);
    ui->controlsGroup->setEnabled(running);
}

void MW::applySettigns()
{
    commThread.setParameters(ui->updateRate->value(),ui->trackingErrorTolerance->value(),ui->positionGain->value());
}
