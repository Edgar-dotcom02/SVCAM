#include <trackingcam3d_reader/MyFpsMeter.h>
#include <opencv/cv.h>

#ifndef _GO_timing
	#ifndef _GET_timing
		#include <stdio.h>
		#include <iostream>
		#include <iomanip>
		#define _GO_timing(x)	double x = (double)cvGetTickCount();	//Save operation start time time/
		#define _GET_timing(x)	{x = ((double)cvGetTickCount() - x)/(cvGetTickFrequency()*1000.); std::cout << "Time of " << #x << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << x << " ms " << std::endl;}
	#endif
#endif

MyFpsMeter::MyFpsMeter()
{
	tick_count_ = cvGetTickCount();
	name_ = "";
	fps_filter_k_ = 1;
	fps_ = 0;
}

MyFpsMeter::MyFpsMeter(std::string name)
{
	tick_count_ = cvGetTickCount();
	name_ = name;
	fps_filter_k_ = 1;
	fps_ = 0;
}

MyFpsMeter::MyFpsMeter(std::string name, double fps_filter_k)
{
	tick_count_ = cvGetTickCount();
	name_ = name;
	fps_filter_k_ = fps_filter_k;
	fps_ = 0;
}


MyFpsMeter::~MyFpsMeter()
{
}

double MyFpsMeter::update()
{
	int64_t new_tick_count = cvGetTickCount();
	period_ = ((double)new_tick_count - tick_count_)/(cvGetTickFrequency()*1000.); 
	fps_ = fps_ * (1 - fps_filter_k_) + (1000 / period_) * fps_filter_k_;
	tick_count_ = new_tick_count;
	
	return fps_;
}

void MyFpsMeter::print_period(bool is_force_update)
{
	if(is_force_update)
		update();
	std::cout << "Time of " << name_ << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << period_ << " ms " << std::endl;
}

void MyFpsMeter::print_here(std::string label, bool is_force_update)
{
	if(is_force_update)
		update();
	std::cout << "Time of " << label << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << period_ << " ms " << std::endl;
}

void MyFpsMeter::print_fps(bool is_force_update)
{
	if(is_force_update)
		update();
	std::cout << "Frequency of " << name_ << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(1) << fps_ << " Hz" << std::endl;
}