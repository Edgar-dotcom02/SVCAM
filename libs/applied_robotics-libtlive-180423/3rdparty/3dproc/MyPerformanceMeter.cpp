#include "MyPerformanceMeter.h"
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

namespace tlive {

MyPerformanceMeter::MyPerformanceMeter()
{
	name_ = " ";
	is_print_on_stop_ = true;
	last_label_ = name_;
	is_enabled_ = true;
	go(name_);
	is_async_job_wait_callback_ = false;
}

MyPerformanceMeter::MyPerformanceMeter(std::string name, bool is_enabled,  bool is_print_on_stop)
{
	name_ = name;
	is_print_on_stop_ = is_print_on_stop;
	last_label_ = name;
	is_enabled_ = true;
	go(name_);
	is_enabled_ = is_enabled;
	is_async_job_wait_callback_ = false;
}

MyPerformanceMeter::MyPerformanceMeter(std::string name, bool is_enabled, bool is_print_on_stop, void (*async_job_wait_callback)(void))
{
	name_ = name;
	is_print_on_stop_ = is_print_on_stop;
	last_label_ = name;
	is_enabled_ = true;
	go(name_);
	is_enabled_ = is_enabled;
	is_async_job_wait_callback_ = false;
	set_async_job_wait_callback(async_job_wait_callback);
}

MyPerformanceMeter::~MyPerformanceMeter()
{
}

void MyPerformanceMeter::go(std::string label)
{
	if(!is_enabled_)
		return;
	if(!counters_.count(label))
		order_.push_back(label);
	counters_[label].delta = 0;
	counters_[label].period = 0;
	last_label_ = label;
	
	counters_[label].tick_count = cvGetTickCount();
}

double MyPerformanceMeter::stop(std::string label)
{
	if(!is_enabled_)
		return 0;
	if(is_async_job_wait_callback_)
		async_job_wait_callback_();
	int64_t cur = cvGetTickCount();
	counters_[label].delta = cur - counters_[label].tick_count;
	counters_[label].period = ((double)counters_[label].delta) / (cvGetTickFrequency()*1000.);

	if(is_print_on_stop_)
		print(label);

	return counters_[label].period;
}

double MyPerformanceMeter::stop(std::string label, void (*async_job_wait_callback)(void))
{
	if(!is_enabled_)
		return 0;
	async_job_wait_callback();
	return stop(label);
}

double MyPerformanceMeter::stop(void (*async_job_wait_callback)(void))
{
	if(!is_enabled_)
		return 0;
	return stop(last_label_, async_job_wait_callback);
}

double MyPerformanceMeter::stop()
{
	if(!is_enabled_)
		return 0;
	return stop(last_label_);
}

double MyPerformanceMeter::get(std::string label)
{
	if(!is_enabled_)
		return 0;
	double period = stop(label);
	print(label);
	return period;
}

void MyPerformanceMeter::print(std::string label)
{
	if(!is_enabled_)
		return;
	std::cout << "Time of " << label << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << counters_[label].period << " ms " << std::endl;
}

void MyPerformanceMeter::print_all()
{
	if(!is_enabled_)
		return;
	std::cout << "Performance test " << name_ << ":" << std::endl;
	//std::cout << "  " << "Overall meter << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << counters_[order_[0]].period << " ms " << std::endl;
	for(int i = 1; i < order_.size(); i++)
	{
		std::cout << "  " << order_[i] << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << counters_[order_[i]].period << " ms " << std::endl;
	}
}

std::string MyPerformanceMeter::string_all()
{
    if(!is_enabled_)
        return "";

    std::stringstream ss;
    ss << "Performance test " << name_ << ":" << std::endl;
    //std::cout << "  " << "Overall meter << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << counters_[order_[0]].period << " ms " << std::endl;
    for(int i = 1; i < order_.size(); i++)
    {
        ss << "  " << order_[i] << " = " << std::setiosflags(std::ios::fixed) << std::setprecision(4) << counters_[order_[i]].period << " ms " << std::endl;
    }
    return ss.str();
}

void MyPerformanceMeter::set_async_job_wait_callback(void (*async_job_wait_callback)(void))
{
	async_job_wait_callback_ = async_job_wait_callback;
	is_async_job_wait_callback_ = true;
}


};