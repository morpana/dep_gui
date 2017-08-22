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

	motors = 14;
	sensors = 14;

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

	updateMatrixList(ui->matrixList);
	updateMatrixList(ui->matrixList_2);

	QObject::connect(ui->loadMatrix, SIGNAL(released()), this, SLOT(restoreMatrix()));

	//function creation
	QObject::connect(ui->step, SIGNAL(released()), this, SLOT(addStep()));
	QObject::connect(ui->ramp, SIGNAL(released()), this, SLOT(addRamp()));
	QObject::connect(ui->sine, SIGNAL(released()), this, SLOT(addSine()));

	QObject::connect(ui->saveFunction, SIGNAL(released()), this, SLOT(saveFunction()));
	QObject::connect(ui->initFunction, SIGNAL(released()), this, SLOT(initFunction()));
	QObject::connect(ui->removeFunction, SIGNAL(released()), this, SLOT(removeFunctionFromList()));
	QObject::connect(ui->editFunction, SIGNAL(released()), this, SLOT(editFunctionFromList()));

	//plotting created function
	// create graph for each func
	ui->plotFunction->addGraph();
	
	ui->plotFunction->xAxis->setLabel("time");
	ui->plotFunction->yAxis->setLabel("y");
	
	ui->plotFunction->xAxis->setRange(0.0, 1.0);
	ui->plotFunction->yAxis->setRange(-0.5, 1.5);

	// matrix vector buttons
	QObject::connect(ui->addMatrix, SIGNAL(released()), this, SLOT(addMatrix()));
	QObject::connect(ui->removeMatrix, SIGNAL(released()), this, SLOT(removeMatrix()));

	//Plotting DEP matrix again	

	ui->customPlot_2->xAxis->setLabel("motor");
	ui->customPlot_2->yAxis->setLabel("sensor");
	ui->customPlot_2->xAxis->setRange(0,motors);
	ui->customPlot_2->yAxis->setRange(0,sensors);

	colorMap1 = new QCPColorMap(ui->customPlot_2->xAxis, ui->customPlot_2->yAxis);
	colorMap1->setGradient(QCPColorGradient::gpPolar);

	colorMap1->data()->setSize(motors, sensors); // we want the color map to have motors * sensors data points
	colorMap1->data()->setRange(QCPRange(0, motors), QCPRange(0, sensors)); // and span the coordinate range 0..(motors|sensors)
	colorMap1->setDataRange(dataRange);
	colorMap1->setInterpolate(false);

	colorScale = new QCPColorScale(ui->customPlot_2);
	ui->customPlot_2->plotLayout()->addElement(0, 1, colorScale); // add it to the right of the main axis rect
	colorScale->setType(QCPAxis::atRight); // scale shall be vertical bar with tick/axis labels right (actually atRight is already the default)
	colorMap1->setColorScale(colorScale); // associate the color map with the color scale
	colorScale->axis()->setLabel("Connection strength");
	colorScale->setDataRange(dataRange);

	QCPMarginGroup *marginGroup1 = new QCPMarginGroup(ui->customPlot_2);
	ui->customPlot_2->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup1);
	colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup1);
	
	ui->customPlot_2->rescaleAxes();

	QObject::connect(ui->plotMatrix, SIGNAL(released()), this, SLOT(plotMatrix()));	
	QObject::connect(ui->plotMatrixSelected, SIGNAL(released()), this, SLOT(plotMatrixSelected()));

	QObject::connect(ui->pubLinComb, SIGNAL(released()), this, SLOT(togglePubLinComb()));

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
/*
void MainWindow::plotDepMatrix_2(){

	for (int i = 0; i < C.size(); i++) {
		for (int j = 0; j < C[0].size(); j++) {
			colorMap->data()->setCell(i,j,C[i][j]);
		}
	}
	ui->customPlot->replot();
}*/

void MainWindow::publishLinearCombination(){
	while (1){
		if (publish_combination){
			roboy_dep::depMatrix msg;
			matrix::Matrix temp_matrix = lin.out(time_);
			msg.size = temp_matrix.getM();
			for (int i = 0; i < msg.size; i++) {
				roboy_dep::cArray msg_temp;
				msg_temp.size = temp_matrix.getN();
				for (int j = 0; j < msg_temp.size; j++) {
					msg_temp.cArray.push_back(temp_matrix.val(i,j));
				}
				msg.depMatrix.push_back(msg_temp);
			}
			depLoadMatrix.publish(msg);
			time_ += 0.02;
		}
		usleep(20000);
	}
}

void MainWindow::togglePubLinComb(){
	//check correct number of matrices and functions specified
	if ((ui->selectedMatrixList->count() != 0) and (ui->functionList->count() != 0) and (ui->selectedMatrixList->count() == ui->functionList->count())){	
		ui->warning->setText("");
		publish_combination = (publish_combination != true);
		if (publish_combination){
			ui->combinationLabel->setText("Combination: Enabled");
			learning = false;
			ui->learningLabel->setText("Learning: Disabled");
		} else {
			ui->combinationLabel->setText("Combination: Disabled");
			learning = true;
			ui->learningLabel->setText("Learning: Enabled");
		}
	} else {
		ui->warning->setText("Must have non-zero equal number of functions and matrices!");
	}
}

void MainWindow::plotMatrixSelected(){
	if (ui->selectedMatrixList->selectedItems().size() != 0){
		int k = atoi(ui->selectedMatrixList->currentItem()->text().toStdString().c_str());
		for (int i = 0; i < lin.m[k].getM(); i++) {
			for (int j = 0; j < lin.m[k].getN(); j++) {
				colorMap1->data()->setCell(i,j,lin.m[k].val(i,j));
			}
		}
		ui->customPlot_2->replot();
	} else {

	}
}

void MainWindow::plotMatrix(){

	roboy_dep::depMatrix msg;
	if (ui->matrixList_2->selectedItems().size() != 0){
		string filename = ui->matrixList_2->currentItem()->text().toStdString();
		ifstream file(directory+filename);

		if (file.is_open()){
			string value;
			roboy_dep::cArray msg_temp;
			char delim = ',';
			while(getline(file,value,delim)){
				string row_end = "NEW_ROW";
				if (value.c_str() != row_end){
					msg_temp.cArray.push_back(atof(value.c_str()));
				} else {
					msg_temp.size = msg_temp.cArray.size();
					msg.depMatrix.push_back(msg_temp);
					msg_temp.cArray.clear();
				}
			}
			msg.size = msg.depMatrix.size();
		}
		file.close();

		// assign the data from the depMatrix
		for (int i = 0; i < msg.size; i++) {
			for (int j = 0; j < msg.depMatrix[i].size; j++) {
				colorMap1->data()->setCell(i,j,msg.depMatrix[i].cArray[j]);
			}
		}
		ui->customPlot_2->replot();
	} else {

	}
}

void MainWindow::removeMatrix(){
	if (ui->selectedMatrixList->selectedItems().size() != 0){
		int i = atoi(ui->selectedMatrixList->currentItem()->text().toStdString().c_str());
		lin.m.erase(lin.m.begin()+i);
		ui->selectedMatrixList->clear();
		for (int i; i < lin.m.size(); i++){
			ui->selectedMatrixList->addItem(to_string(i).c_str());
		}
	}
}

void MainWindow::addMatrix(){

	roboy_dep::depMatrix msg;
	if (ui->matrixList_2->selectedItems().size() != 0){
		string filename = ui->matrixList_2->currentItem()->text().toStdString();
		ifstream file(directory+filename);

		if (file.is_open()){
			string value;
			roboy_dep::cArray msg_temp;
			char delim = ',';
			while(getline(file,value,delim)){
				string row_end = "NEW_ROW";
				if (value.c_str() != row_end){
					msg_temp.cArray.push_back(atof(value.c_str()));
				} else {
					msg_temp.size = msg_temp.cArray.size();
					msg.depMatrix.push_back(msg_temp);
					msg_temp.cArray.clear();
				}
			}
			msg.size = msg.depMatrix.size();
		}
		file.close();

		matrix::Matrix temp = matrix::Matrix(msg.size, msg.depMatrix[0].size);
		// assign the data from the depMatrix
		for (int i = 0; i < msg.size; i++) {
			for (int j = 0; j < msg.depMatrix[i].size; j++) {
				temp.val(i,j) = msg.depMatrix[i].cArray[j];
			}
		}
		
		lin.addMatrix(temp);
		ui->selectedMatrixList->clear();
		for (int i; i < lin.m.size(); i++){
			ui->selectedMatrixList->addItem(to_string(i).c_str());
		}
	} else {

	}
}

void MainWindow::editFunctionFromList(){
	if (ui->functionList->selectedItems().size() != 0){
		int i = atoi(ui->functionList->currentItem()->text().toStdString().c_str());
		temp_function = lin.f[i];
		lin.f.erase(lin.f.begin()+i);
		updateFunctionList();
		plotFunc();
	} else {

	}
}

void MainWindow::removeFunctionFromList(){
	if (ui->functionList->selectedItems().size() != 0){
		int i = atoi(ui->functionList->currentItem()->text().toStdString().c_str());
		lin.f.erase(lin.f.begin()+i);
		updateFunctionList();
	} else {

	}
}

void MainWindow::updateFunctionList(){
	// list widget showing available functions
	ui->functionList->clear();
	for (int i; i < lin.f.size(); i++){
		ui->functionList->addItem(to_string(i).c_str());
	}
}

void MainWindow::initFunction(){
	temp_function.clear();
	temp_function.setPeriod(stod(ui->period->text().toStdString().c_str()));
	ui->plotFunction->xAxis->setRange(0.0, temp_function.getPeriod());
	plotFunc();
}

void MainWindow::saveFunction(){
	lin.addFunction(temp_function);
	temp_function.clear();
	plotFunc();
	updateFunctionList();
}

void MainWindow::plotFunc(){
	int steps = (int) temp_function.getPeriod()/0.01+0.5;
	QVector<double> time(steps+1), y(steps+1);
	for (int i = 0; i < steps+1; i++){
		time.append(i*0.01);
		y.append(temp_function.out(i*0.01));
	}
	ui->plotFunction->graph(0)->setData(time, y);
	ui->plotFunction->replot();
}

void MainWindow::addStep(){
	//need to fix period setting!!!
	double amp = stod(ui->step_amp->text().toStdString().c_str());
	double cutoff = stod(ui->step_cutoff->text().toStdString().c_str());
	step_ step;
	step.setParams(cutoff,amp);
	temp_function.addStep(step);

	double t_on = stod(ui->t_on->text().toStdString().c_str());
	double t_off = stod(ui->t_off->text().toStdString().c_str());
	window_ window;
	window.setParams(t_on, t_off);
	temp_function.addWindow(window);
	
	plotFunc();
}

void MainWindow::addRamp(){
	double slope = stod(ui->ramp_slope->text().toStdString().c_str());
	double shift = stod(ui->ramp_shift->text().toStdString().c_str());
	ramp_ ramp;
	ramp.setParams(slope,shift);
	temp_function.addRamp(ramp);

	double t_on = stod(ui->t_on->text().toStdString().c_str());
	double t_off = stod(ui->t_off->text().toStdString().c_str());
	window_ window;
	window.setParams(t_on, t_off);
	temp_function.addWindow(window);
	
	plotFunc();
}

void MainWindow::addSine(){
	double amp = stod(ui->sine_amp->text().toStdString().c_str());
	double freq = stod(ui->sine_freq->text().toStdString().c_str());
	double phase = stod(ui->sine_phase->text().toStdString().c_str());

	sine_ sine;
	sine.setParams(amp,freq,phase);
	temp_function.addSine(sine);

	double t_on = stod(ui->t_on->text().toStdString().c_str());
	double t_off = stod(ui->t_off->text().toStdString().c_str());
	window_ window;
	window.setParams(t_on, t_off);
	temp_function.addWindow(window);
	
	plotFunc();
}

void MainWindow::toggleLearning(){
	if (!publish_combination){
		learning = (learning != true); //perform XOR
		if (learning){
			ui->learningLabel->setText("Learning: Enabled");
		} else {
			ui->learningLabel->setText("Learning: Disabled");
		}
		//ROS_INFO("%i", learning);
		setDepConfig();
	} else {
		ui->learningLabel->setText("Turn off publishing!");
	}
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
		file.open(directory+filename+extension, ios::trunc);
		for (int i = 0; i < C.size(); i++) {
			string s;
			for (int j = 0; j < C[0].size(); j++) {
				s.append(boost::lexical_cast<std::string>(C[i][j])+",");
			}
			file << s << "NEW_ROW,";
		}
		file.close();
		updateMatrixList(ui->matrixList);
		updateMatrixList(ui->matrixList_2);
		ui->feedback->setText("Input filename");
		updateMatrixList(ui->matrixList);
		updateMatrixList(ui->matrixList_2);
	} else{
		ui->feedback->setText("No C matrix to save!");
	}
}

void MainWindow::updateMatrixList(QListWidget* list){
	// list widget showing available matrices
	list->clear();
	QDir matrix_directory(QString::fromStdString(directory));
	QStringList matrices = matrix_directory.entryList();
	regex e ("(.*)(.dep)");
	for (int i; i < matrices.size(); i++){
		if (regex_match(matrices[i].toStdString().c_str(),e)){
			//ROS_INFO("%s",matrices[i].toStdString().c_str());
			list->addItem(matrices[i].toStdString().c_str());
		}
	}
}

void MainWindow::restoreMatrix(){
	roboy_dep::depMatrix msg;

	//string filename = ui->loadFile->text().toStdString();
	if (ui->matrixList->selectedItems().size() != 0){
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
	} else {

	}
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