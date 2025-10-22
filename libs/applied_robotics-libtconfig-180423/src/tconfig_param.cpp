#include "tconfig_param.h"
#include <cfloat>

#ifdef WIN32
#	include <windows.h>
#	include <tchar.h>
#	define PATH_SEPARATOR "\\"
#else
#	include <limits.h>
#	include <stdlib.h>
#	define PATH_SEPARATOR "/"
#endif

static const char delimiters[] = {' ', ',', '\n', '\0', ';'};

namespace tconfig
{

static void print_full_path(const char* _filenamev_)
{
#ifdef WIN32
	TCHAR _full_path_[MAX_PATH]; 
	wchar_t wmsg[MAX_PATH + 1]; 
	mbstowcs(wmsg, _filenamev_, strlen(_filenamev_) + 1); 
	GetFullPathName(wmsg, MAX_PATH, _full_path_, NULL); 
	_tprintf(_T("%s\n"), _full_path_);
#else
	char _full_path_[PATH_MAX];
	realpath(_filenamev_, _full_path_); 
	printf("%s\n", _full_path_);
#endif
}

bool is_param_in_map(std::map<std::string, Param> &params, std::string name)
{
	return params.find(name) != params.end();
}

bool is_delimiter(char c)
{
	for(int i = 0; i < sizeof(delimiters); i++)
		if(delimiters[i] == c)
			return true;
	return false;
}

void Param::sprintf_auto_precision(char* buf, double val)
{
	sprintf(buf, "%.15f", val);
	char* p = buf + strlen(buf) - 1;
	while(p >= buf && (*p == '0' || *p == ' '))
		p--;

	char* d = strchr(buf, '.');

	int precision = p - d;
	if(precision == 0)
		sprintf(buf, "%lld", (long long int)val);
	else
	{
		//p[1] = ' ';
		p[2] = '\0';
	}
}

bool Param::check(Param val)
{
	if(strlen(lim_str_) > 0) 
	{
		std::string vstr = val.str();
		if(val.str().length() < 1)
			return false;
		const char* cs = vstr.c_str();
		char* p = strstr(lim_str_, cs);
			
		if(p != NULL)
		{
			p += strlen(cs) - 1;
			if(is_delimiter(p[1]))
				return true;
		}
	}
	else if(get_vec_size() > 1)
	{
		std::vector<double> v = dvec(); 
		for(int i = 0; i < v.size(); i++) 
			if(!(v[i] > min_ - std::numeric_limits<double>::epsilon() && v[i] < max_ + std::numeric_limits<double>::epsilon() )) 
				return false; 
		return true;
	}
	else if(val >= min_ && val <= max_) 
		return true;
	return false;
}

bool Param::fw(std::string word)
{
	return str_.find(word) != std::string::npos;
}

void clean_str(char* buf)
{
	char* b = buf;

	//Replace all delimiters with spaces
	while(*b != '\n')
		if(is_delimiter(*b))
			*b = ' ';
	b = buf;

	//Trim delimiters on beginning
	while(*b == ' ' && *b != '\0')
		b++;

	//delete all duplicates
	char* d = buf;
	while(*b != '\0')
	{
		if(b[0] == ' ' && b[1] == ' ')
			continue;
		*(d++) = *(b++);
	}
}

char* trim_str(char *buf) 
{
	char *ptr;
	// remove head spaces
	while(*buf && isspace(*buf))
		buf++;
	// remove section
	if(buf[0] == '[') 
	{
		buf[0] = '\0';
		return buf;
	}
	// remove comments
	if((ptr = strchr(buf, '#')) != NULL)
		*ptr = '\0';
	if((ptr = strchr(buf, ';')) != NULL)
		*ptr = '\0';
	if((ptr = strchr(buf, '/')) != NULL) 
	{
		if(*(ptr+1) == '/')
			*ptr = '\0';
	}
	// move ptr to the end, again
	for(ptr = buf; *ptr; ptr++)
		;
	--ptr;
	// remove comments
	while(ptr >= buf) 
	{
		if(*ptr == '#')
			*ptr = '\0';
		ptr--;
	}
	// move ptr to the end, again
	for(ptr = buf; *ptr; ptr++)
		;
	--ptr;
	// remove tail spaces
	while(ptr >= buf && isspace(*ptr))
		*ptr-- = '\0';
	//
	return buf;
}

/**
* Parse a single configuration string. This is an internal function.
*
* @param filename [in] The current configuration file name.
* @param lineno [in] The current line number of the string.
* @param buf [in] The content of the current configuration string.
* @return 0 on success, or -1 on error.
*/
int parse_param(char *buf, std::string &name, std::string &value)
{
	char *option, *token; //, *saveptr;
	char *leftbracket, *rightbracket;
	//gaConfVar gcv;
	//
	option = buf;
	if((token = strchr(buf, '=')) == NULL) 
	{
		return 0;
	}
	if(*(token + 1) == '\0') 
	{
		return 0;
	}
	*token++ = '\0';
	//
	option = trim_str(option);
	if(*option == '\0')
		return 0;
	//
	token = trim_str(token);
	if(token[0] == '\0')
		return 0;

	name = option;
	value = token;
	return 0;
}

int parse_params(std::map<std::string, Param> &params, char* buf, bool is_add_new)
{
	char cur_buf[8192];
	int cur_buf_i = 0;
	char* p = buf;


	while(1)
	{
		while(*p != '=' && *p != '\0')
			p++;

		if(*p == '=')
		{
			p++;
			while(*p != '=' && *p != '\0')
				p++;

			if(*p == '=')
			{
				p--;
				while(*p == ' ' && *p != '=')
					p--;
				while(*p != ' ')
					p--;
			}
		}
		else
			break;

		int option_len = p - buf;
		memcpy(cur_buf, buf, option_len);
		cur_buf[option_len] = '\0';
		buf = p;

		std::string name, value;
		if(parse_param(cur_buf, name, value))
			continue;
		if(is_param_in_map(params, name) || is_add_new)
			params[name] = value;
	}

	return 0;
}

int parse_get_reqs(std::vector<std::string> &get_reqs, char* buf)
{
	char* p = buf + 1;
	int len = strlen(buf);
	for(int i = 1; i < len; i++)
	{
		if(buf[i] == '?' && (buf[i + 1] == ' ' || buf[i + 1] == '\0' || buf[i + 1] == '\n'))
		{
			int a = i - 1;
			while(a > 0 && buf[a - 1] != ' ')
				a--;
			std::string name(buf + a, i - a);
			get_reqs.push_back(name);
		}
	}

	return 0;
}

void put_param(std::string &dst, std::string name, int val)
{
	char tmp[21];
	sprintf(tmp, "%d", val);
	dst += name + "=" + tmp + " ";
}

void put_param_as_hex(std::string &dst, std::string name, int val)
{
	char tmp[21];
	sprintf(tmp, "0x%08x", val);
	dst += name + "=" + tmp + " ";
}


void put_param(std::string &dst, std::string name, std::string val)
{
	char tmp[21];
	dst += name + "=" + val + " ";
}

bool get_param(std::map<std::string, Param> &params, std::string name, Param &dst)
{
	std::map<std::string, Param>::iterator it;
	it = params.find(name);
	if(it == params.end())
		return false;

	dst = it->second;
	return true;
}

int TextClass::parse_config_file(const char* filename, bool is_load_unknowns)
{
	FILE *fp;
	char buf[8192];
	int lineno = 0;
	//
	if(filename == NULL)
		return -1;
	if((fp = fopen(filename, "rt")) == NULL) 
	{
		return -1;
	}
	while(fgets(buf, sizeof(buf), fp) != NULL) 
	{
		lineno++;
		if(parse_params(params, buf, is_load_unknowns) < 0) 
		{
			fclose(fp);
			return -1;
		}
	}
	fclose(fp);
	return lineno;
}


TextClass::~TextClass(void)
{
}

bool TextClass::is_my_param(std::string param_name)
{
	return is_param_in_map(params, param_name);
}

int TextClass::hash_params() 
{
	int h = 0; 
	for(std::map<std::string, Param>::iterator it = params.begin(); it != params.end(); it++) 
		h += it->second.hash(); 
	return h;
}


int TextClass::load_params(const char* filename, bool is_load_unknowns)
{
	if(parse_config_file(filename, is_load_unknowns) >= 0)
	{
        //printf("Config file %s is loaded successfully.\n", filename);
	}
	else
	{
		printf("Cant load config file %s. Full path:\n", filename);
		print_full_path(filename);
		return -1;
	}
	return 0;
}

void TextClass::fput_param(FILE* f, std::string name)
{
	fprintf(f, "%s = %s\n", name.c_str(), params[name].str().c_str());
}

int TextClass::save_params(const char* filename)
{
	FILE* f = fopen(filename, "w");
	if(f == NULL)
	{
		printf("Cant open file %s for writing. Full path:\n", filename);
		print_full_path(filename);
		return -1;
	}

	for(std::map<std::string, Param>::iterator it = params.begin(); it != params.end(); it++)
		fput_param(f, it->first);

	fclose(f);

	//printf("Config %s is saved. Full path:\n", filename);
	//PRINT_FULL_PATH(filename);
	return 0;
}

int TextClass::sprintf_params(std::string &str)
{
	for(std::map<std::string, Param>::iterator it = params.begin(); it != params.end(); it++)
		str += it->first + " = " + it->second.str() + "\n";
	return 0;
}

int TextClass::printf_params()
{
	std::string str;
	sprintf_params(str);
	printf("%s", str.c_str());
	return 0;
}

bool TextClass::raid_params()
{
	bool is_somthing_new = false;
	for(std::map<std::string, Param>::iterator it = params.begin(); it != params.end(); it++)
	{
		is_somthing_new = is_somthing_new || (bool)it->second.raid();
	}
	return is_somthing_new;
}

std::vector<int> str_to_ints(std::string str, int n)
{
    std::vector<int> vals;
    int reads = 0;
    const char *ptr = str.c_str();
    char *endptr;
    while(reads < n)
    {
        vals.push_back(strtol(ptr, &endptr, 0));
        if(ptr == endptr)
            break;
        ptr = endptr;
        reads++;
    }
    return vals;
}

std::string ints_to_str(std::vector<int> vals)
{
    std::string s = "";
    for(int i = 0; i < vals.size(); i++)
    {
        char buf[100];
        sprintf(buf, "%d", vals[i]);
        s += std::string(buf);
        if(i != vals.size() - 1)
            s += " ";
    }
    return s;
}

double str_to_double(std::string str)
{
    const char* p = str.c_str();
    return strtod(p, NULL);
}

std::string double_to_str(double val)
{
    char buf[100];
    sprintf(buf, "%1.5f", val);
    return buf;
}

std::string str_no_delimiters(std::string s)
{
	for(std::string::iterator it = s.begin(); it != s.end(); it++)
	{
		if(*it == ' ')
			*it = '_';
		else if(*it == '\n')
			*it = '|';
	}
	return s;
}

};