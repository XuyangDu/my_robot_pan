/************************************************************************

 About: A class to manage the config file
 Author: MinCheng
 Date: 2011/09/30
 File: ConfigManager.h


 ************************************************************************/

#pragma  once
#include <string>
#include <map>
using std::string;
using std::map;

typedef map<string, string> ConfigListType;

class ConfigManager
{
public:
  ConfigManager();
  ConfigManager(const string _fileName);
  void setFileName(const string _fileName);
  bool loadConfigs();

  template<class T>
    bool getValue(const char* tag, T& value);
  template<class T>
    bool setValue(const char* tag, T value);

  bool saveConfigs();
  const string& getErrorTex();

private:
  bool processLine(const string & line, string& tag, string& value);
  void normString(string& str);
  template<class T>
    bool convertFromString(T &value, const std::string &s);
  template<class T>
    bool convertToString(T &value, std::string &s);

  string fileName;
  bool bChanged;
  bool bLoaded;
  ConfigListType configList;
  string errorTex;
};

template<class T>
  bool ConfigManager::getValue(const char* tag, T& value)
  {
    if (bLoaded == false)
    {
      errorTex = "Did not load file";
      return false;
    }

    ConfigListType::iterator it = configList.find(tag);
    if (it == configList.end())
    {
      errorTex = "Tag '";
      errorTex += tag;
      errorTex += "' ";
      errorTex += "does not exist";
      return false;
    }

    return convertFromString(value, it->second);
  }

template<class T>
  bool ConfigManager::setValue(const char* tag, T value)
  {
    string valueString;
    if (convertToString(value, valueString))
    {
      ConfigListType::iterator it = configList.find(tag);
      if (it == configList.end())
        configList.insert(ConfigListType::value_type(tag, valueString));
      else
        configList[tag] = valueString;
    }
    else
      return false;

    bChanged = true;
    return true;

  }

template<class T>
  bool ConfigManager::convertFromString(T &value, const std::string &s)
  {
    std::stringstream ss(s);
    ss >> value;
    if (!ss.fail())
      return true;
    else
      return false;
  }

template<class T>
  bool ConfigManager::convertToString(T &value, std::string &s)
  {
    std::stringstream ss;
    ss << value;
    s = ss.str();
    if (!ss.fail())
      return true;
    else
      return false;
  }

