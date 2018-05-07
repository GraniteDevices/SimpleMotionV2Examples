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
    commThread.start();
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
}

void MW::on_resetDriveFaults_clicked()
{
    commThread.clearDriveErrors();
}

void MW::on_updateRate_valueChanged(int arg1)
{
    applySettigns();
}

void MW::on_positionGain_valueChanged(double arg1)
{
    applySettigns();
}

void MW::on_maxSpeed_valueChanged(int value)
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

void MW::applySettigns()
{
    commThread.setParameters(ui->updateRate->value(),ui->trackingErrorTolerance->value(),ui->positionGain->value(),ui->maxSpeed->value());
}
