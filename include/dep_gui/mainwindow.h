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

#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>
#include <stdlib.h>

#include <regex>

#include <dep_gui/linearCombination.h>

//#include <pthread.h>
#include <thread>

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
	std::thread spawn_thread(){
		return std::thread( [this] {this->publishLinearCombination();});
	}

private:
	Ui::MainWindow *ui;
	ros::NodeHandlePtr nh;
	ros::Publisher depCommand, motorConfig, depParameters, depLoadMatrix;
	ros::Subscriber depMatrix;
	boost::shared_ptr<ros::AsyncSpinner> spinner;

	vector<vector<double>> C;

	int motors, sensors;
	//ros::Subscriber sub;
	//void Print(const roboy_communication_middleware::MotorConfig::ConstPtr &msg);
	//void Print(const roboy_dep::depParameters::ConstPtr &msg);
	//void printMatrix(const roboy_dep::depMatrix::ConstPtr &msg);
	//void msgLoadDepMatrix(const roboy_dep::depMatrix::ConstPtr &msg);
	QCPColorMap *colorMap, *colorMap1;
	QCPColorScale *colorScale;
	void msgDepMatrix(const roboy_dep::depMatrix::ConstPtr &msg);
	void updateMatrixList(QListWidget* list);
	bool learning;
	string directory;
	function_ temp_function;
	linearCombination lin;
	void plotFunc();
	void updateFunctionList();
	
	double time_ = 0;
	bool publish_combination = false;
	void publishLinearCombination(); 

Q_SIGNALS:
	void newDepMatrix();
private Q_SLOTS:
	void togglePubLinComb();
	void plotMatrix();
	void plotMatrixSelected();
	void removeMatrix();
	void addMatrix();
	void editFunctionFromList();	
	void removeFunctionFromList();
	void initFunction();
	void saveFunction();
	void sendCommand(QString);
	void setMotorConfig();
	void setDepConfig();
	void plotDepMatrix();
	void toggleLearning();
	void storeMatrix();
	void restoreMatrix();
	void addStep();
	void addRamp();
	void addSine();
};

#endif // MAINWINDOW_H


//sudo kill -9 $(ps -al|grep '\(roboy_dep\|dep_interface\|dep_gui\)'|awk '{print $4;}'|tr '\n' ' ')