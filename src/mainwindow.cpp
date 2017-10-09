#include "dep_gui/mainwindow.h"
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
#include <algorithm>
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
	motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &MainWindow::MotorStatus, this);
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
	sensors = 14*2;
	current_matrix = matrix::Matrix(motors,sensors);
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
	QObject::connect(ui->removeMatrix_2, SIGNAL(released()), this, SLOT(removeMatrix_2()));

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

	QObject::connect(this, SIGNAL(newMotorData()), this, SLOT(plotMotorData()));
	for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
		ui->position_plot->addGraph();
		ui->position_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
		ui->displacement_plot->addGraph();
		ui->displacement_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
	}
	ui->position_plot->xAxis->setLabel("x");
	ui->position_plot->yAxis->setLabel("ticks");
	ui->position_plot->replot();

	ui->displacement_plot->xAxis->setLabel("x");
	ui->displacement_plot->yAxis->setLabel("ticks");
	ui->displacement_plot->replot();

	QObject::connect(ui->removeComponent, SIGNAL(released()), this, SLOT(removeComponent()));	


	//transition stuff
	QObject::connect(ui->stepTransition, SIGNAL(released()), this, SLOT(stepTransition()));	
	QObject::connect(ui->rampTransition, SIGNAL(released()), this, SLOT(rampTransition()));	
	QObject::connect(ui->sineTransition, SIGNAL(released()), this, SLOT(sineTransition()));	

	transition =  {false};
	transition_type = 0;

	QObject::connect(ui->triggerEdge, SIGNAL(released()), this, SLOT(toggleTriggerEdge()));
	QObject::connect(ui->setTrigger, SIGNAL(released()), this, SLOT(toggleTrigger()));
	trigger_level = 0.0;
	trigger_edge = 1;
	//trigger_motor = {0};
	trigger_on = 0;

	transition_pub = nh->advertise<roboy_dep::transition>("/roboy_dep/transition", 1);
	prev_filename = "None";

	time_ = 0.0;
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

void MainWindow::toggleTriggerEdge(){
	trigger_edge = (trigger_edge != true);
	if (trigger_edge == 1){
		ui->edgeText->setText("Rising edge");
	} else {
		ui->edgeText->setText("Falling edge");
	}
}

void MainWindow::toggleTrigger(){
	trigger_on = (trigger_on != true);
	if (trigger_on == 0){
		ui->triggerText->setText("Off");
	} else {
		ui->triggerText->setText("On");
	}

	// obtain trigger_motor vector from delimited list committed by user in GUI
	trigger_motor.clear();	
	trigger_level = atof(ui->triggerLevel->text().toStdString().c_str());
	string s = ui->triggerMotor->text().toStdString();
	std::vector<std::string> words;
	boost::split(words, s, boost::is_any_of(", ;"), boost::token_compress_on);
	for (int i = 0; i < words.size(); i++){
		trigger_motor.push_back(atoi(words[i].c_str()));
		//ROS_INFO("%i", trigger_motor[i]);
	}

}

void MainWindow::stepTransition(){
	transition_type = 0;
}
void MainWindow::rampTransition(){
	transition_type = 1;
	duration = atof(ui->ramp_transition_duration->text().toStdString().c_str());
}
void MainWindow::sineTransition(){
	transition_type = 2;
	duration = atof(ui->sine_transition_duration->text().toStdString().c_str());
}


void MainWindow::plotMotorData(){
	for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
		ui->position_plot->graph(motor)->setData(plot_time, motorPos[motor]);
		ui->displacement_plot->graph(motor)->setData(plot_time, motorForce[motor]);
		if (motor == 0) {
			ui->position_plot->graph(motor)->rescaleAxes();
			ui->displacement_plot->graph(motor)->rescaleAxes();
		} else {
			ui->position_plot->graph(motor)->rescaleAxes(true);
			ui->displacement_plot->graph(motor)->rescaleAxes(true);
		}
	}
	ui->position_plot->replot();
	ui->displacement_plot->replot();
}

void MainWindow::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg){
	for (int i = 0; i < NUMBER_OF_MOTORS_PER_FPGA; i++){
		//ROS_INFO("%i", msg->position[i]);
		motorPos[i].push_back(msg->position[i]);
		motorForce[i].push_back(msg->displacement[i]);
		if(motorPos[i].size() > 300){
			motorPos[i].pop_front();
			motorForce[i].pop_front();
		}
	}
	plot_time.push_back(counter++*0.002);
	if(plot_time.size()>300){
		plot_time.pop_front();
	}
	Q_EMIT newMotorData();
}

void MainWindow::removeMatrix_2(){
	if (	ui->matrixList->selectedItems().size() != 0){
		string s = 	ui->matrixList->currentItem()->text().toStdString();
		delete 	ui->matrixList->takeItem(ui->matrixList->row(ui->matrixList->currentItem()));
		string temp = directory+s;
		ROS_INFO("%s", temp.c_str());
		boost::filesystem::remove(temp.c_str());
	} else {

	}
}

bool isTrue (int i){
	return i == true;
}

void MainWindow::publishLinearCombination(){
	vector<bool> on;
	vector<double> prev_level;
	vector<double> curr_level;
	while (1){
		t_start = std::chrono::high_resolution_clock::now();

		matrix::Matrix temp_matrix;
		if (publish_combination){
			temp_matrix = lin.out(time_);
			roboy_dep::depMatrix msg;
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
		} else if (any_of(transition.begin(),transition.end(),isTrue)){
			ROS_INFO("test");
			// initialization for on, prev_level, curr_level
			while (on.size() != trigger_motor.size()){
				// this is here to reinitialize if muscles are taken out from list
				if (on.size() > trigger_motor.size()){
					on.clear();
					prev_level.clear();
					curr_level.clear();
				}
				on.push_back(false);
				prev_level.push_back(0.0);
				curr_level.push_back(0.0);
			}
			// if trigger enabled, calculate matrix accordingly
			if (trigger_on){
				for(int i=0; i < trigger_motor.size(); i++){
					prev_level[i] = (motorPos[trigger_motor[i]][294]+motorPos[trigger_motor[i]][295]+motorPos[trigger_motor[i]][296])/3;
					curr_level[i] = (motorPos[trigger_motor[i]][297]+motorPos[trigger_motor[i]][298]+motorPos[trigger_motor[i]][299])/3;
					//ROS_INFO("%f, %f", prev_level[i], curr_level[i]);
					if (trigger_edge){
						if (prev_level[i] < trigger_level and curr_level[i] > trigger_level){
							on[i] = 1;
						}
					} else {
						if (prev_level[i] > trigger_level and curr_level[i] < trigger_level){
							on[i] = 1;
						}
					}
					// if the given muscle crosses the threshold, initialize/continue transition process and calculate the desired matrix
					if (on[i]){
						// if not yet initialized, initialize timer
						if (start[i]){
							time_vec[i] = 0.0;
							start[i] = false;
						}
						if (transition_type == 0){
							//calculate new matrix
							temp_matrix = current_matrix;
							for(int j=0; j<temp_matrix.getN(); j++){
								temp_matrix.val(trigger_motor[i],j) = restore_matrix.val(trigger_motor[i],j);
							}
							// need to update current matrix in case another motor is updated before temp_matrix is sent i.e. during the same cycle
							current_matrix = temp_matrix;
							ROS_INFO("Motor: %i", trigger_motor[i]);
							// transition is complete, end transition
							transition[i] = false;
							on[i] = 0;
						} else if (transition_type == 1){
							//calculate new matrix
							double w0 = time_vec[i]/duration;
							double w1 = 1.0-w0;
							temp_matrix = restore_matrix*w0+current_matrix*w1;
							
							//if transition is complete, end transition for muscle
							if (time_vec[i]/duration > 1.0){
								transition[i] = false;
								on[i] = 0;
							}
						} else if (transition_type == 2){
							//calculate new matrix
							double w0 = sin(PI/duration*time_vec[i]-PI/2)/2+0.5;
							double w1 = 1-w0;	
							temp_matrix = restore_matrix*w0+current_matrix*w1;
							
							//if transition is complete, end transition for muscle
							if (time_vec[i]/duration > 1.0){
								transition[i] = false;
								on[i] = 0;
							}
						}
					}
				}
			} else {
				// if the trigger is not enabled, but a transition is called, update full matrix immediately
				temp_matrix = restore_matrix;
				fill(transition.begin(),transition.end(),false);
				fill(start.begin(),start.end(),false);
			}

			// publish new matrix (in either case)
			roboy_dep::depMatrix msg;
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
			ROS_INFO(" ");
		}
		// increment timer (for time dependent transitions i.e. ramp, sine)
		time_ += 0.02;
		for(int i=0;i<time_vec.size();i++){
			time_vec[i] += 0.02;
		}

		// only perform updates at 20 ms intervals, so wait until 20 ms is done
		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
		while(diff < 20){
			usleep(100);
			t_end = std::chrono::high_resolution_clock::now();
			diff = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
		}
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
		int motors = lin.m[k].getM();
		int sensors = lin.m[k].getN();
		for (int i = 0; i < motors; i++) {
			for (int j = 0; j < sensors; j++) {
				colorMap1->data()->setCell(i,j,lin.m[k].val(i,j));
			}
		} 
		ui->customPlot_2->xAxis->setRange(0,motors);
		ui->customPlot_2->yAxis->setRange(0,sensors);
		colorMap1->data()->setRange(QCPRange(0, motors), QCPRange(0, sensors)); // and span the coordinate range 0..(motors|sensors)
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
		int motors = msg.size;
		int sensors = msg.depMatrix[0].size;
		for (int i = 0; i < motors; i++) {
			for (int j = 0; j < sensors; j++) {
				colorMap1->data()->setCell(i,j,msg.depMatrix[i].cArray[j]);
			}
		}
		ui->customPlot_2->xAxis->setRange(0,motors);
		ui->customPlot_2->yAxis->setRange(0,sensors);
		colorMap1->data()->setRange(QCPRange(0, motors), QCPRange(0, sensors)); // and span the coordinate range 0..(motors|sensors)
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

void MainWindow::removeComponent(){
	string s(ui->componentList->currentItem()->text().toStdString());	
	regex ex1 ("^.*?([0-9]+$)");
	regex ex2 ("^([0-9]+?.*$)");
	smatch sm;
	regex_match(s,sm,ex2);
	for (int i = 0; i < sm.size(); i++){
		ROS_INFO("%s", sm[i]);
	}
}

void MainWindow::updateComponentList(){
	ui->componentList->clear();
	int steps = 0;
	int ramps = 0;
	int sines = 0;
	string num;
	for (int i = 0; i < temp_function.f.size(); i++){
		string type = temp_function.f[i]->type();
		if (type == "step"){ steps++; num = to_string(steps);} else if (type == "ramp"){ ramps++; num = to_string(ramps);} else if (type == "sine") { sines++; num = to_string(sines);}
		ui->componentList->addItem((to_string(i)+" - "+type+"_"+num).c_str());
	}
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
	
	updateComponentList();
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
	
	updateComponentList();
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
	
	updateComponentList();
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
	for (int i = 0; i < C.size(); i++) {
		for (int j = 0; j < C[0].size(); j++) {
			current_matrix.val(i,j)= C[i][j];
		}
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
		ROS_INFO("Storing matrix: %s", (filename+extension).c_str());
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
	// if linear combination is not publishing
	if (!publish_combination){
		//string filename = ui->loadFile->text().toStdString();
		
		// if a file is selected
		if (ui->matrixList->selectedItems().size() != 0){
			string filename;
			filename = ui->matrixList->currentItem()->text().toStdString();
			//ROS_INFO("%s", (directory+filename).c_str());
			ifstream file(directory+filename);

			roboy_dep::depMatrix msg;
			if (file.is_open()){
				string value;
				roboy_dep::cArray msg_temp;
				char delim = ',';
				//double d;
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
		
			restore_matrix = matrix::Matrix(msg.size, msg.depMatrix[0].size);
			for (int i = 0; i < msg.size; i++) {
				for (int j = 0; j < msg.depMatrix[i].size; j++) {
					restore_matrix.val(i,j) = msg.depMatrix[i].cArray[j];
				}
			}
			//initialize transition and start
			transition.clear();
			start.clear();
			time_vec.clear();
			for (int i = 0; i < trigger_motor.size(); i++) {
				transition.push_back(true);
				start.push_back(true);
				time_vec.push_back(0.0);
			}

			//publish message simply recording transition details
			roboy_dep::transition msg1;
			msg1.matrix_filename = filename;
			msg1.prev_filename = prev_filename;
			msg1.transition_type = transition_type;
			msg1.trigger_on = trigger_on;
			msg1.trigger_motor = trigger_motor;
			msg1.trigger_level = trigger_level;
			msg1.trigger_edge = trigger_edge;
			msg1.duration = duration;
			transition_pub.publish(msg1);

			prev_filename = filename;
		}
	} else {
		ui->learningLabel->setText("Turn off publishing!");
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
	msg.springMult2 = stod(ui->springMult2->text().toStdString().c_str());
	msg.diff = stod(ui->diff->text().toStdString().c_str());
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