#ifndef MY_FPS_METER_H
#define MY_FPS_METER_H

#include <string>
#include <stdint.h>

class MyFpsMeter
{
	int64_t tick_count_;
public:
	MyFpsMeter();
	MyFpsMeter(std::string name);
	MyFpsMeter(std::string name, double fps_filter_k);
	~MyFpsMeter();

	double period_;
	double fps_;
	double fps_filter_k_;
	std::string name_;

	double update();
	void print_period(bool is_force_update = true);
	void print_fps(bool is_force_update = true);
	void print_here(std::string label = "", bool is_force_update = true);
};

#endif