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
	depLoadMatrix = nh->advertise<roboy_dep::depMatrix>("/roboy_dep/depLoadMatrix", 1);

	depMatrix = nh->subscribe("/roboy_dep/depMatrix", 1, &MainWindow::msgDepMatrix, this); 
	//sub = nh->subscribe("/roboy_dep/depLoadMatrix", 1, &MainWindow::msgLoadDepMatrix, this);
	//sub = nh->subscribe("/roboy_dep/depParameters", 1, &MainWindow::Print, this);

	ui->setupUi(this);

	QObject::connect(this, SIGNAL(newDepMatrix()), this, SLOT(plotDepMatrix()));

	QSignalMapper* signalMapper = new QSignalMapper(this);

	QObject::connect(ui->force, SIGNAL(released()), signalMapper, SLOT(map()));
	QObject::connect(ui->init, SIGNAL(released()), signalMapper, SLOT(map()));
	QObject::connect(ui->update, SIGNAL(released()), signalMapper, SLOT(map()));

	signalMapper -> setMapping(ui->force, "force");
	signalMapper -> setMapping(ui->init, "init");
	signalMapper -> setMapping(ui->update, "update");

	QObject::connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(sendCommand(QString)));

	QObject::connect(ui->toggleLearning, SIGNAL(released()), this, SLOT(toggleLearning()));
	QObject::connect(ui->updateController, SIGNAL(released()), this, SLOT(setMotorConfig()));
	QObject::connect(ui->updateDepParams, SIGNAL(released()), this, SLOT(setDepConfig()));

	QObject::connect(ui->saveMatrix, SIGNAL(released()), this, SLOT(storeMatrix()));

	int motors = 14;
	int sensors = 14;

	// configure axis rect:
	ui->customPlot->xAxis->setLabel("motor");
	ui->customPlot->yAxis->setLabel("sensor");
	ui->customPlot->xAxis->setRange(0,motors);
	ui->customPlot->yAxis->setRange(0,sensors);

	//Plotting DEP matrix
	colorMap = new QCPColorMap(ui->customPlot->xAxis, ui->customPlot->yAxis);
	colorMap->setGradient(QCPColorGradient::gpPolar);

	colorMap->data()->setSize(motors, sensors); // we want the color map to have motors * sensors data points
	colorMap->data()->setRange(QCPRange(0, motors), QCPRange(0, sensors)); // and span the coordinate range 0..(motors|sensors)
	const QCPRange& dataRange = QCPRange(-0.5,0.5);
	colorMap->setDataRange(dataRange);
	colorMap->setInterpolate(false);

	// add a color scale:
	colorScale = new QCPColorScale(ui->customPlot);
	ui->customPlot->plotLayout()->addElement(0, 1, colorScale); // add it to the right of the main axis rect
	colorScale->setType(QCPAxis::atRight); // scale shall be vertical bar with tick/axis labels right (actually atRight is already the default)
	colorMap->setColorScale(colorScale); // associate the color map with the color scale
	colorScale->axis()->setLabel("Connection strength");
	colorScale->setDataRange(dataRange);
	
	// make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
	QCPMarginGroup *marginGroup = new QCPMarginGroup(ui->customPlot);
	ui->customPlot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
	colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
	 
	// rescale the key (x) and value (y) axes so the whole color map is visible:
	ui->customPlot->rescaleAxes();

	spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(5));
	spinner->start();

	learning = 1;

	string temp(getenv("HOME"));
	directory = temp;
	directory.append("/dep_matrices/");
	const char* path = directory.c_str();
	struct stat sb;
	if (stat(path, &sb) == 0 && S_ISDIR(sb.st_mode)){
	} else {
		boost::filesystem::path dir(path);
		boost::filesystem::create_directory(dir);
	}

	updateMatrixList();
	QObject::connect(ui->loadMatrix, SIGNAL(released()), this, SLOT(restoreMatrix()));
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

void MainWindow::toggleLearning(){
	learning = (learning != true); //perform XOR
	if (learning){
		ui->learningLabel->setText("Learning: Enabled");
	} else {
		ui->learningLabel->setText("Learning: Disabled");
	}
	//ROS_INFO("%i", learning);
	setDepConfig();
}

/*
void MainWindow::msgLoadDepMatrix(const roboy_dep::depMatrix::ConstPtr &msg){
	for (int i = 0; i < msg->size; i++) {
		std::string s;
		for (int j = 0; j < msg->depMatrix[i].size; j++) {
			s.append(boost::lexical_cast<std::string>(msg->depMatrix[i].cArray[j])+",");
		}
		ROS_INFO("%s", s.c_str());
	}
}*/

void MainWindow::msgDepMatrix(const roboy_dep::depMatrix::ConstPtr &msg){
	C.clear();
	// assign the data from the depMatrix
	for (int i = 0; i < msg->size; i++) {
		vector<double> temp;
		//std::string s;
		for (int j = 0; j < msg->depMatrix[i].size; j++) {
			temp.push_back(msg->depMatrix[i].cArray[j]);
			//s.append(boost::lexical_cast<std::string>(msg->depMatrix[i].cArray[j])+",");
		}
		C.push_back(temp);
		//ROS_INFO("%s", s.c_str());
	}
	// plot data
	Q_EMIT newDepMatrix();
}

void MainWindow::plotDepMatrix(){
	for (int i = 0; i < C.size(); i++) {
		for (int j = 0; j < C[0].size(); j++) {
			colorMap->data()->setCell(i,j,C[i][j]);
		}
	}
	ui->customPlot->replot();
}


void MainWindow::storeMatrix(){
	if (!C.empty()){
		string filename = ui->saveFile->text().toStdString();
		//ROS_INFO("%s", (directory+filename).c_str());
		string extension = ".dep";
		ofstream file;
		file.open(directory+filename+extension);
		for (int i = 0; i < C.size(); i++) {
			string s;
			for (int j = 0; j < C[0].size(); j++) {
				s.append(boost::lexical_cast<std::string>(C[i][j])+",");
			}
			file << s << "NEW_ROW,";
		}
		file.close();
		updateMatrixList();
		ui->feedback->setText("Input filename");
		updateMatrixList();
	} else{
		ui->feedback->setText("No C matrix to save!");
	}
}

void MainWindow::updateMatrixList(){
	// list widget showing available matrices
	ui->matrixList->clear();
	QDir matrix_directory(QString::fromStdString(directory));
	QStringList matrices = matrix_directory.entryList();
	regex e ("(.*)(.dep)");
	for (int i; i < matrices.size(); i++){
		if (regex_match(matrices[i].toStdString().c_str(),e)){
			//ROS_INFO("%s",matrices[i].toStdString().c_str());
			ui->matrixList->addItem(matrices[i].toStdString().c_str());
		}
	}
}

void MainWindow::restoreMatrix(){
	roboy_dep::depMatrix msg;

	//string filename = ui->loadFile->text().toStdString();
	string filename = ui->matrixList->currentItem()->text().toStdString();
	//ROS_INFO("%s", (directory+filename).c_str());
	ifstream file(directory+filename);

	if (file.is_open()){
		string value;
		roboy_dep::cArray msg_temp;
		char delim = ',';
		//double d;
		while(getline(file,value,delim)){
			string row_end = "NEW_ROW";
			if (value.c_str() != row_end){
				//d += 0.3;
				//if (d>0.5){d -= 1;}
				//msg_temp.cArray.push_back(d);
				msg_temp.cArray.push_back(atof(value.c_str()));
				//ROS_INFO("%s",value.c_str());
			} else {
				/*
				for (int i = 0; i < msg_temp.size; i++){
					ROS_INFO("%f", msg_temp[i]);
				}*/
				//ROS_INFO("\n");
				msg_temp.size = msg_temp.cArray.size();
				msg.depMatrix.push_back(msg_temp);
				msg_temp.cArray.clear();
			}

		}
		msg.size = msg.depMatrix.size();
	}
	//ROS_INFO("%i, %i", msg.size, msg.depMatrix[0].size);
	file.close();
	depLoadMatrix.publish(msg);
}

void MainWindow::sendCommand(QString s){
	//QString s = ui->textEdit->toPlainText();
	setDepConfig();
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
	msg.learning = learning;
	msg.targetForce = atoi(ui->targetForce->text().toStdString().c_str());
	msg.range = atoi(ui->range->text().toStdString().c_str());
	depParameters.publish(msg);
	//double urate = stod(ui->urate->text().toStdString().c_str());
	//cout << urate << "\n";
}