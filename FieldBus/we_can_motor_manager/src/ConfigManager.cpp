/************************************************************************

 About: A class to manage the config file
 Author: MinCheng
 Date: 2011/09/30
 File: ConfigManager.cpp


 ************************************************************************/
#include <fstream>
#include <string>
#include <sstream>
#include "ConfigManager.h"

using namespace std;

typedef map<string, string> ConfigListType;

ConfigManager::ConfigManager()
{
  bChanged = false;
  bLoaded = false;

}

ConfigManager::ConfigManager(const string _fileName)
{
  this->fileName = _fileName;
  bChanged = false;
  bLoaded = false;
}

void ConfigManager::setFileName(const string _fileName)
{
  this->fileName = _fileName;
}

bool ConfigManager::loadConfigs()
{
  if (fileName.empty())
  {
    errorTex = "File name not set.";
    return false;
  }
  ifstream file(fileName.c_str());
  if (!file)
  {
    errorTex = "File does not exist.";
    return false;

  }

  string line;
  string tag;
  string value;
  while (getline(file, line))
  {
    if (line[0] == '#' || line[0] == '[')
      continue;
    if (processLine(line, tag, value))
      configList.insert(ConfigListType::value_type(tag, value));

  }
  file.close();
  bLoaded = true;
  return true;
}

bool ConfigManager::saveConfigs()
{
  if (bChanged == false)
    return true;

  if (fileName.empty())
  {
    errorTex = "File name not set.";
    return false;
  }

  ofstream file;
  file.open(fileName.c_str());
  ConfigListType::iterator it;
  for (it = configList.begin(); it != configList.end(); it++)
  {
    file << it->first << ": " << it->second << endl;

  }

  file.close();
  bChanged = false;

  return true;
}

const string& ConfigManager::getErrorTex()
{
  return errorTex;
}

bool ConfigManager::processLine(const string & line, string& tag, string& value)
{
  string::size_type middle = line.find_first_of(':', 0);
  if (middle == string::npos)
    return false;
  tag = line.substr(0, middle - 0);
  value = line.substr(middle + 1, line.length() - middle - 1);
  normString(tag);
  normString(value);
  return true;
}

void ConfigManager::normString(string& str)
{
  string::size_type begin = str.find_first_not_of(' ', 0);
  string::size_type end = str.find_last_not_of(' ');
  if (begin != string::npos && end != string::npos)
    str = str.substr(begin, end - begin + 1);
}
