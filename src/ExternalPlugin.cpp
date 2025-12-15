#include "HYYRobotInterface.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <iostream>
#include <map>
//
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 外部功能开启
 * @param name 名称（确保唯一）
 * @param arg 输入参数(注：使用时与函数内自定义实现用的数据类型保持一致)
 * @param return 返回错误代码(错误代码自行定义)
 */
extern int ExternalPluginOpen(const char* name, void* arg);

/**
 * @brief 外部功能操作
 * @param name 名称(与接口保持一致)
 * @param value 输入参数(注：使用时与函数内自定义实现用的数据类型保持一致)
 * @param return 返回错误代码(错误代码自行定义)
 */
extern int ExternalPluginFunction(const char* name, void* value);

/**
 * @brief 外部功能关闭(与接口保持一致)
 * @param name 名称(与接口保持一致)
 * @param return 返回错误代码(错误代码自行定义)
 */
extern int ExternalPluginClose(const char* name);

#ifdef __cplusplus
}
#endif

//------------以下用户根据需要自行设计----------------

//根据需要自行定义参数结构,此处为demo
struct ExternalArg{
    int arg1;
    int arg2;
};

struct ExternalValue{
    int type;
    int value;
};

class ExternalDemo
{
public:
    ExternalDemo(){};
    ~ExternalDemo(){};
    int Init(struct ExternalArg* arg);
    int FuncitionIn(int value);
    int FuncitionOut(int *value);
private:

};

int ExternalDemo::Init(struct ExternalArg* arg)
{
    printf("ExternalDemo::Init:%d,%d\n",arg->arg1,arg->arg2);
    return 0;
}

int ExternalDemo::FuncitionIn(int value)
{
    printf("ExternalDemo::FuncitionIn:%d\n",value);
    return 0;
}

int ExternalDemo::FuncitionOut(int *value)
{
    *value=100;//返回数据
    printf("ExternalDemo::FuncitionOut:%d\n",*value);
    return 0;
}


static std::map<std::string, ExternalDemo*> _data;


int ExternalPluginOpen(const char* name, void* arg)
{
    std::string name_s(name);
    if (_data.find(name_s) == _data.end())
    {
        _data[name_s]=new ExternalDemo();

        struct ExternalArg earg=*((struct ExternalArg*)arg);

        return _data[name_s]->Init(&earg);
    }
    return -1;

}

int ExternalPluginFunction(const char* name, void* value)
{

    std::string name_s(name);
    if (_data.find(name_s) != _data.end())
    { 
        int ret=0;
        struct ExternalValue* v=(struct ExternalValue*)value;
        switch (v->type)
        {
        case 0:
            ret=_data[name_s]->FuncitionIn(v->value);
            break;
        case 1:
            ret=_data[name_s]->FuncitionOut(&(v->value));
            break;
        }
        return ret;
    }
    return -1;
}


int ExternalPluginClose(const char* name)
{
    std::string name_s(name);
    if (_data.find(name_s) != _data.end())
    {
        delete _data[name_s];
        _data.erase(name_s);
        return 0;
    }
    return -1;
}