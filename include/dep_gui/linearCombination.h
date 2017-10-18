#ifndef LINCOMB_H
#define LINCOMB_H

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include "dep_gui/matrix.h"

#define PI 3.14159265

using namespace std;

class timeFunction {
	public:
		virtual double out(double time) = 0;
		virtual string type() = 0;
};

class step_: public timeFunction {
	public:
		double out(double time){ if (time > t) { return 1.0*A; } else { return 0.0; } }
		void setParams(double cutoff, double amplitude){ t = cutoff; A = amplitude;}
		string type(){return "step";}
	private:
		double t, A;
};

class window_: public timeFunction {
	public:
		double out(double time){ return ON.out(time) + OFF.out(time); }
		void setParams(double t_start, double t_end){ 
			t_on = t_start; t_off = t_end; 
			ON.setParams(t_on, 1.0); OFF.setParams(t_off, -1.0);
		}
		string type(){return "window";}
	private:
		double t_on, t_off;
		step_ ON, OFF;
};

class ramp_: public timeFunction {
	public:
		double out(double time){ return m*(time-phi); }
		void setParams(double slope, double phase){ 
			m = slope; phi = phase; 
		}
		string type(){return "ramp";}
	private:
		double m, phi;
};

class sine_: public timeFunction {
	public:
		double out(double time){ return A*sin(2*PI*f*time+phi); }
		void setParams(double amplitude, double frequency, double phase){ 
			A = amplitude; f = frequency; phi = phase; 
		}
		string type(){return "sine";}
	private:
		double A, f, phi;
};

class function_ {
	public:
		void setPeriod(double period){ T = period; }
		double getPeriod(){ return T;}
		void addStep(step_ s){
			steps.push_back(s);
			//ROS_INFO("%p",&steps[steps.size()-1]);
			//f.push_back(&steps[steps.size()-1]);
			//ROS_INFO("%p",f[f.size()-1]);
		}
		void addRamp(ramp_ r){
			ramps.push_back(r);
			genF();
		}
		void addSine(sine_ s){
			sines.push_back(s);
			genF();
		}
		void addWindow(window_ win){
			w.push_back(win);
			genF();
		}
		void clear(){
			steps.clear();
			ramps.clear();
			sines.clear();
			f.clear();
			w.clear();
		}
		void genF(){
			f.clear();
			for (int i = 0; i < steps.size(); i++){
				f.push_back(&steps[i]);
			}
			for (int i = 0; i < ramps.size(); i++){
				f.push_back(&ramps[i]);
			}
			for (int i = 0; i < sines.size(); i++){
				f.push_back(&sines[i]);
			}
		}
		double out(double time){
			double weight = 0;//, fnc, win;
			genF();
			for (int i = 0; i < f.size(); i++){
				//cout << "i: " << i << "\n";
				//cout << "f[i]: " << f[i] << "\n";
				//cout << "f[i]->out(time): " << f[i]->out(time) << "\n";
				//fnc = f[i]->out(fmod(time,T));
				//win = w[i].out(fmod(time,T));
				//cout << fnc << " " << win << "\n";
				weight += (f[i]->out(fmod(time,T)))*(w[i].out(fmod(time,T)));
			}
			return weight;
		}
		vector<timeFunction*> f;
	private:
		double T;
		vector<step_> steps;
		vector<ramp_> ramps;
		vector<sine_> sines;
		vector<window_> w;
};

class linearCombination {
	public:
		void addFunction(function_ func){
			f.push_back(func);
		}
		void addMatrix(matrix::Matrix matr){
			m.push_back(matr);
		}
		roboy_dep::linear_combination pub(double time){
			roboy_dep::linear_combination msg;
			for (int i = 0; i < m.size(); i++){
				msg.weights.push_back(f[i].out(time));
			}
			return msg;
		}
		matrix::Matrix out(double time){
			matrix::Matrix y = matrix::Matrix(m[0].getM(),m[0].getN());
			for (int i = 0; i < m.size(); i++){
				y = y + m[i]*f[i].out(time);
			}
			return y;
		}
		vector<function_> f;
		vector<matrix::Matrix> m;
};

/*
int main(){
	//create functions and functions vector
	step_ step;
	step.setParams(0.4, 1.0);
	ramp_ ramp;
	ramp.setParams(1.0,0.0);
	sin_ sin;
	sin.setParams(1.0,1.0,0.0);

	vector<timeFunction*> functions;
	functions.push_back(&step);
	functions.push_back(&ramp);
	functions.push_back(&sin);
	
	//create windows for functions and windows vector
	window_ window1;
	window1.setParams(0.25, 0.75);
	window_ window2;
	window2.setParams(0.25, 0.75);
	window_ window3;
	window3.setParams(0.25, 0.75);
	
	vector<window_> windows;
	windows.push_back(window1);
	windows.push_back(window2);
	windows.push_back(window3);
	
	//create functions
	function_ f;
	f.setPeriod(1.0);
	f.setFunctions(functions);
	f.setWindows(windows);
	vector<function_> funcs;
	funcs.push_back(f);

	//create matrices
	matrix::Matrix m = matrix::Matrix(1,1);
	m.val(0,0) = 1.0;
	vector<matrix::Matrix> matrices;
	matrices.push_back(m);

	//calulate matrix combination at time = 0.6
	matrix::Matrix y = matrix::Matrix(1,1);
	y = linearCombination(matrices,funcs,0.6);
	cout << y.val(0,0);
}*/

#endif