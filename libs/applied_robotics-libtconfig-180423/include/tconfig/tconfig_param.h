#ifndef LIBTCONFIG_PARAM_H
#define LIBTCONFIG_PARAM_H

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <iterator>
#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <cfloat>

namespace tconfig
{

class Param
{
	std::string str_;
	int hash_;
	bool is_raided_;
	
	char* lim_str_;
	double min_;
	double max_;

	void set_i(int i) {char buf[31]; sprintf(buf, "%d", i); str_ = buf; update();}
	void set_d(double d) {char buf[31]; sprintf_auto_precision(buf, d); str_ = buf; update();}
	void set_str(std::string s) {str_ = s; update();}
	void set_b(bool b) {str_ = b? "1": "0"; update();}
	void set_ivec(std::vector<int> ivec) {str_ = ""; for(unsigned int i = 0; i < ivec.size(); i++) {char buf[31]; sprintf(buf, "%d ", ivec[i]); str_ += buf;} update();};
	void set_dvec(std::vector<double> dvec) {str_ = ""; for(unsigned int i = 0; i < dvec.size(); i++) {char buf[100]; sprintf_auto_precision(buf, dvec[i]); str_ += std::string(buf) + " ";} update();};
	
	int get_vec_size(){std::stringstream stream(str_); return std::distance(std::istream_iterator<std::string>(stream), std::istream_iterator<std::string>());};
	
	void unlimit() {lim_str_ = ""; min_ = -DBL_MAX; max_ = DBL_MAX;}
	void strlimit(char* s) {unlimit(); lim_str_ = s;}
	void dlimit(double min, double max) {unlimit(); min_ = min; max_ = max;}

	void update() {hash_++; is_raided_ = false;};
	
	template<typename T> std::vector<T> parse_vector() {std::stringstream iss(str_); std::vector<T> vec; T val; while (iss >> val) vec.push_back(val); return vec;};
	void sprintf_auto_precision(char* buf, double val);
	bool fw(std::string word);
public:
	Param(): str_("") {unlimit(); update();};
	Param(int i) {unlimit(); set_i(i);};
	Param(double d) {unlimit(); set_d(d);};
	Param(std::string str): str_(str) {unlimit();};
	Param(char* str): str_(str) {unlimit();};
	Param(bool b) {unlimit(); set_b(b);};
	Param(std::vector<int> ivec) {unlimit(); set_ivec(ivec);};
	Param(std::vector<double> dvec) {unlimit(); set_dvec(dvec);};

	Param(int i, char* str_acceptable_vals) {strlimit(str_acceptable_vals); set_i(i);};
	Param(double d, char* str_acceptable_vals) {strlimit(str_acceptable_vals); set_d(d);};
	Param(std::string str, char* str_acceptable_vals): str_(str) {strlimit(str_acceptable_vals);};
	Param(char* str, char* str_acceptable_vals): str_(str) {strlimit(str_acceptable_vals);};
	Param(std::vector<int> ivec, char* str_acceptable_vals) {strlimit(str_acceptable_vals); set_ivec(ivec);};
	Param(std::vector<double> dvec, char* str_acceptable_vals) {strlimit(str_acceptable_vals); set_dvec(dvec);};

	Param(double min, double max): str_("") {dlimit(min, max);};
	Param(int i, double min, double max) {dlimit(min, max); set_i(i);};
	Param(double d, double min, double max) {dlimit(min, max); set_d(d);};
	Param(std::string str, double min, double max): str_(str) {dlimit(min, max);};
	Param(char* str, double min, double max): str_(str) {dlimit(min, max);};
	Param(std::vector<int> ivec, double min, double max) {dlimit(min, max); set_ivec(ivec);};
	Param(std::vector<double> dvec, double min, double max) {dlimit(min, max); set_dvec(dvec);};

	~Param(){};
	bool operator	>	(double a) {return d() > a + std::numeric_limits<double>::epsilon();}
	bool operator	>=	(double a) {return (*this > a || *this == a);}
	bool operator	==	(double a) {return std::fabs(d() - a) <= std::numeric_limits<double>::epsilon();}
	bool operator	!=	(double a) {return !(*this == a);}
	bool operator	<	(double a) {return d() < a - std::numeric_limits<double>::epsilon();}
	bool operator	<=	(double a) {return (*this < a || *this == a);}
	double operator +	(double a) {return d() + a;}
	double operator -	(double a) {return d() - a;}
	double operator *	(double a) {return d() * a;}
	double operator /	(double a) {return d() / a;}
	bool operator	=	(double a) {return set(a);}
	bool operator	+=	(double a) {return set(d() + a);}
	bool operator	-=	(double a) {return set(d() - a);}
	bool operator	*=	(double a) {return set(d() * a);}
	bool operator	/=	(double a) {return set(d() / a);}
	bool set(double a) {if(!check(a)) return false; set_d(a); return true;}

	bool operator	>	(int a) {return i() > a;}
	bool operator	>=	(int a) {return i() >= a;}
	bool operator	==	(int a) {return i() == a;}
	bool operator	!=	(int a) {return i() != a;}
	bool operator	<	(int a) {return i() < a;}
	bool operator	<=	(int a) {return  i() <= a;}
	int operator	+	(int a) {return i() + a;}
	int operator	-	(int a) {return i() - a;}
	int operator	*	(int a) {return i() * a;}
	int operator	/	(int a) {return i() / a;}
	bool operator	=	(int a) {return set(a);}
	bool operator	+=	(int a) {return set(i() + a);}
	bool operator	-=	(int a) {return set(i() - a);}
	bool operator	*=	(int a) {return set(i() * a);}
	bool operator	/=	(int a) {return set(i() / a);}
	bool set(int a) {if(!check(a)) return false; set_i(a); return true;}
		
	bool operator	==	(std::string a) {return str_ == a;}
	bool operator	!=	(std::string a) {return str_ != a;}
	bool operator	=	(std::string a) {return set(a);}
	bool operator	=	(char* a)		{return set(std::string(a));}
	bool set(std::string a) {if(!check(a)) return false; set_str(a); return true;}
		
	//bool operator > (double a) {std::vector<double> v = dvec(); for(int i = 0; i < v.size(); i++) if(!(v[i] > a + std::numeric_limits<double>::epsilon())) return false; return true;}
	bool operator	==	(std::vector<double> a) {std::vector<double> v = dvec(); for(unsigned int i = 0; i < v.size(); i++) if(std::fabs(v[i] - a[i]) <= std::numeric_limits<double>::epsilon()) return false; return true;}
	bool operator	!=	(std::vector<double> a) {return !(*this == a);}
	bool operator	=	(std::vector<double> a) {return set(a);}
	bool set(double* a, int n)		{std::vector<double> v(a, a + n); return set(v);}
	bool set(std::vector<double> a) {if(!check(a)) return false; set_dvec(a); return true;}

	//bool operator >= (int a) {std::vector<int> v = ivec(); for(int i = 0; i < v.size(); i++) if(!(v[i] >= a)) return false; return true;}
	bool operator	==	(std::vector<int> a) {std::vector<int> v = ivec(); for(unsigned int i = 0; i < v.size(); i++) if(!(v[i] == a[i])) return false; return true;}
	bool operator	!=	(std::vector<int> a) {return !(*this == a);}
	bool operator	=	(std::vector<int> a) {return set(a);}
	bool set(int* a, int n)		 {std::vector<int> v(a, a + n); return set(v);}
	bool set(std::vector<int> a) {if(!check(a)) return false; set_ivec(a); return true;}

	std::string str() {return str_;};
	int i() {return strtol(str_.c_str(), NULL, 0);}
	double d() {return strtod(str_.c_str(), NULL);}
	bool b() {if(strtol(str_.c_str(), NULL, 0)) return true; return fw("true") || fw("TRUE") || fw("True") || fw("enable") || fw("Enable") || fw("ENABLE") || fw("y") || fw("Y") || fw("on") || fw("On") || fw("ON");}
	std::vector<int> ivec(){return parse_vector<int>();}
	std::vector<double> dvec(){return parse_vector<double>();}

	operator std::string() {return str();}
	operator int() {return i();}
	operator double() {return d();}
	operator bool() {return b();}
	operator std::vector<int>() {return ivec();}
	operator std::vector<double>() {return dvec();}

	int hash() {return hash_;}
	Param* raid(Param** p = 0) {Param* ret = is_raided_? 0: this; is_raided_ = true; if(p) *p = ret; return ret;}

	bool check(Param val);
};

class TextClass
{
	int parse_param(char *buf, bool load_unknowns = false);
	int parse_config_file(const char* filename, bool load_unknowns = false);
	void fput_param(FILE* f, std::string name);
public:

	//std::string name;
	std::map<std::string, Param> params;

	TextClass(void)/*: name("")*/{};
	~TextClass(void);
	int save_params(const char* filename);
	int load_params(const char* filename, bool load_unknowns = false);
	int sprintf_params(std::string &str);
	int printf_params();
	bool is_my_param(std::string param_name);
	Param &operator [] (std::string param_name) {return params[param_name];}
	int hash_params();
	bool raid_params();
};

char* trim_str(char *buf);
void clean_str(char* buf);
void clean_str(std::string &str);

std::vector<int> str_to_ints(std::string str, int n);
std::string ints_to_str(std::vector<int> vals);
double str_to_double(std::string str);
std::string double_to_str(double val);
std::string str_no_delimiters(std::string s);

bool is_param_in_map(std::map<std::string, Param> &params, std::string name);
int parse_param(char *buf, std::string &name, std::string &value);
int parse_params(std::map<std::string, Param> &params, char* buf, bool is_add_new = true);
int parse_get_reqs(std::vector<std::string> &get_reqs, char* buf);

void put_param(std::string &dst, std::string name, int val);
void put_param_as_hex(std::string &dst, std::string name, int val);
void put_param(std::string &dst, std::string name, std::string val);

//Get param from map if exists
bool get_param(std::map<std::string, Param> &params, std::string name, Param &dst);

};


#endif
