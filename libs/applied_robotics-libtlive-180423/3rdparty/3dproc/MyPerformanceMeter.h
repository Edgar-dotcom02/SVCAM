#ifndef MY_PERFORMANCE_METER_H
#define MY_PERFORMANCE_METER_H

#include <opencv/cv.h>
#include <string>
#include <map>
#include <stdint.h>
#include <vector>

namespace tlive {

class MyPerformanceMeter
{
	struct Counter
	{
		int64 tick_count;
		int64 delta;
		double period;
	};

	std::map<std::string, Counter> counters_;
	std::vector<std::string> order_;

	bool is_async_job_wait_callback_;
	void (*async_job_wait_callback_)(void);
public:
	MyPerformanceMeter();
	MyPerformanceMeter(std::string name, bool is_enabled = true, bool is_print_on_stop = true);
	MyPerformanceMeter(std::string name, bool is_enabled, bool is_print_on_stop, void (*async_job_wait_callback)(void));
	~MyPerformanceMeter();

	bool is_enabled_;
	bool is_print_on_stop_;
	std::string last_label_;

	double period_;
	std::string name_;

	double update();
	void print_period(bool is_force_update = true);
	void print_fps(bool is_force_update = true);
	void print_here(std::string label = "", bool is_force_update = true);

	void go(std::string label);
	double stop();
	double stop(std::string label);
	double stop(std::string label, void (*async_job_wait_callback)(void));
	double stop(void (*async_job_wait_callback)(void));
	double get(std::string label);
	void print(std::string label);
	void print_all();
    std::string string_all();

	void set_async_job_wait_callback(void (*async_job_wait_callback)(void));
};

};

#endif
