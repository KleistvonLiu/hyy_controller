/**
 * @file PluginTemplate.h
 *
 * @brief 插件接口
 * @author hanbing
 * @version 12.2.0
 * @date 2024-04-18
 */

#ifndef PLUGINTEMPLATE_H_
#define PLUGINTEMPLATE_H_

//声明c++调用接口
#ifdef __cplusplus
//.....
class PluginClassDemo
{
public:
    PluginClassDemo(){};
    ~PluginClassDemo(){};
    void Init();
    void UserClass();
private:

};

PluginClassDemo* GetPluginInstantiation();




#endif
#endif /*PLUGINTEMPLATE_H_*/