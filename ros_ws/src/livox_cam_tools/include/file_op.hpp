/**
 * @file: file_op.hpp
 * @brief: some operations about writing and reading files
 * @details:
 * @author: CHEN Feiyi
 * @email: chenfeiyi@unity-drive.com
 * @version: 1.0.0
 * @licence: Copyright 2021 Unity-Drive Inc. All rights reserved
 * @date:2021-08-30
 */
#pragma once
#include <string.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "dirent.h"

class FileOp
{
public:
  FileOp(/* args */) = default;
  ~FileOp() = default;
  /**
   * @brief: get all files' name under specified folder
   * @param folder_path　the folder you want to check
   * @param file_list　the container that contains all files' name
   * @param concate　true; concate file name with folder，false:
   * do not concate, just file's name.
   * @param verbose
   * @return None
   */
  static void GetFilelist(std::string folder_path,
                          std::vector<std::string> *file_list,
                          bool concate = true, bool verbose = false)
  {
    DIR *dir;
    dirent *ptr;
    std::size_t pos;
    file_list->clear();
    dir = opendir(folder_path.c_str());
    pos = folder_path.find_last_of("/\\");
    if (pos != (folder_path.size() - 1))
      folder_path.push_back('/');

    while ((ptr = readdir(dir)) != NULL)
    {
      if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
        continue;
      std::stringstream ss;
      if (concate)
      {
        ss << folder_path << ptr->d_name;
      }
      else
      {
        ss << ptr->d_name;
      }
      file_list->emplace_back(ss.str());
      if (verbose)
      {
        std::cout << ss.str() << std::endl;
      }
      std::sort(file_list->begin(), file_list->end());
    }
    closedir(dir);
  }
  /**
   * @brief:spilt string with specified seperation character
   * @param str: the string you want to spilt
   * @param field: output containers that contains all substring after spilting
   * @param separators:　seperator
   * @param verbose
   * @return None
   */
  static void SplitString(std::string str, std::vector<std::string> *fields,
                          char separators, bool verbose = false)
  {
    fields->clear();
    std::istringstream sin(str);
    std::string field;
    while (std::getline(sin, field, separators))
    {
      fields->push_back(field);
      if (verbose)
      {
        std::cout << field << "--";
      }
    }
    if (verbose)
      std::cout << std::endl;
  }
  static bool Readtxt(std::string file_path, std::vector<std::string> *fields)
  {
    std::ifstream fin;
    fin.open(file_path.c_str(), std::ios::in);
    if (!fin.is_open())
    {
      std::cout << "can't open file %s" << file_path.c_str() << std::endl;
      fin.close();
      return false;
    }
    std::string line;
    while (std::getline(fin, line))
    {
      fields->push_back(line);
      // SplitString(line, fields, separator, true);
    }
    fin.close();
    return true;
  }
};
