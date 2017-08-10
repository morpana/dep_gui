#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QtGui/QMainWindow>
#include <QtGlobal>
#include "ui_mainwindow.h"
#include <iostream>
#include "ros/ros.h"
#include "roboy_dep/command.h"
#include "roboy_dep/depParameters.h"
#include "roboy_dep/cArray.h"
#include "roboy_dep/depMatrix.h"
#include <roboy_communication_middleware/MotorConfig.h>

#define NUMBER_OF_MOTORS_PER_FPGA 14

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
	Ui::MainWindow *ui;
	ros::NodeHandlePtr nh;
	ros::Publisher depCommand, motorConfig, depParameters;
	ros::Subscriber depMatrix;
	boost::shared_ptr<ros::AsyncSpinner> spinner;

	//ros::Subscriber sub;
	//void Print(const roboy_communication_middleware::MotorConfig::ConstPtr &msg);
	//void Print(const roboy_dep::depParameters::ConstPtr &msg);
	void printMatrix(const roboy_dep::depMatrix::ConstPtr &msg);
private Q_SLOTS:
    void sendCommand(QString);
    void setMotorConfig();
    void setDepConfig();
};

#endif // MAINWINDOW_H
