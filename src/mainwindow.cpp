#include "dep_gui/mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	std::setlocale(LC_NUMERIC, "C");

	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "dep_interface");
	nh = ros::NodeHandlePtr(new ros::NodeHandle);

	depCommand = nh->advertise<roboy_dep::command>("/roboy_dep/command", 1);
	motorConfig = nh->advertise<roboy_communication_middleware::MotorConfig>("/roboy/middleware/MotorConfig", 1);
	depParameters = nh->advertise<roboy_dep::depParameters>("/roboy_dep/depParameters", 1);

	//sub = nh->subscribe("/roboy/middleware/MotorConfig", 1, &MainWindow::Print, this);
	//sub = nh->subscribe("/roboy_dep/depParameters", 1, &MainWindow::Print, this);

	ui->setupUi(this);
	QObject::connect(ui->sendCommand, SIGNAL(released()), this, SLOT(sendCommand()));
	QObject::connect(ui->updateController, SIGNAL(released()), this, SLOT(setMotorConfig()));
	QObject::connect(ui->updateDepParams, SIGNAL(released()), this, SLOT(setDepConfig()));

	spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(5));
	spinner->start();
}

MainWindow::~MainWindow()
{
	delete ui;
}

/*
void MainWindow::Print(const roboy_communication_middleware::MotorConfig::ConstPtr &msg){
	cout << msg->Kp[0] << "\n";
}
*/

/*
void MainWindow::Print(const roboy_dep::depParameters::ConstPtr &msg){
	cout << msg->timedist << "\n";
}
*/

void MainWindow::sendCommand(){
	QString s = ui->textEdit->toPlainText();
	roboy_dep::command msg;
	//cout << qPrintable(s);
	msg.command = qPrintable(s);
	depCommand.publish(msg);
}

void MainWindow::setMotorConfig() {
	roboy_communication_middleware::MotorConfig msg;
	for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
		msg.motors.push_back(motor);
		msg.control_mode.push_back(ui->control_mode->value());
		int outputMax = atoi(ui->outputMax->text().toStdString().c_str());
		if (outputMax >= 0 && outputMax <= 4000) {
			msg.outputPosMax.push_back(outputMax); // pwm max
			msg.outputNegMax.push_back(-outputMax); // pwm min
		} else {
			msg.outputPosMax.push_back(1000); // pwm max
			msg.outputNegMax.push_back(-1000); // pwm min
		}
		msg.spPosMax.push_back(100000000);
		msg.spNegMax.push_back(-100000000);
		msg.IntegralPosMax.push_back(100);
		msg.IntegralNegMax.push_back(-100);
		msg.Kp.push_back(atoi(ui->Kp->text().toStdString().c_str()));
		msg.Ki.push_back(atoi(ui->Ki->text().toStdString().c_str()));
		msg.Kd.push_back(atoi(ui->Kd->text().toStdString().c_str()));
		msg.forwardGain.push_back(0);
		msg.deadBand.push_back(atoi(ui->deadBand->text().toStdString().c_str()));
	}
	motorConfig.publish(msg);
	//cout << ui->control_mode->value();
}

void MainWindow::setDepConfig(){
	roboy_dep::depParameters msg;
	msg.timedist = atoi(ui->timedist->text().toStdString().c_str());
	msg.urate = stod(ui->urate->text().toStdString().c_str());  
	msg.initFeedbackStrength = stod(ui->initFeedbackStrength->text().toStdString().c_str());
	msg.regularization = atoi(ui->regularization->text().toStdString().c_str());
	msg.synboost = stod(ui->synboost->text().toStdString().c_str());	
	msg.maxSpeed = stod(ui->maxSpeed->text().toStdString().c_str());
	msg.epsh = stod(ui->epsh->text().toStdString().c_str());
	msg.pretension = stod(ui->pretension->text().toStdString().c_str());
	msg.maxForce   = stod(ui->maxForce->text().toStdString().c_str());
	msg.springMult1 = stod(ui->springMult1->text().toStdString().c_str());
	msg.delay = atoi(ui->delay->text().toStdString().c_str());
	msg.guideType = atoi(ui->guideType->text().toStdString().c_str());
	msg.guideIdx = atoi(ui->guideIdx->text().toStdString().c_str());
	msg.guideAmpl = stod(ui->guideAmpl->text().toStdString().c_str());
	msg.guideFreq = stod(ui->guideFreq->text().toStdString().c_str());
	depParameters.publish(msg);
	//double urate = stod(ui->urate->text().toStdString().c_str());
	//cout << urate << "\n";
}