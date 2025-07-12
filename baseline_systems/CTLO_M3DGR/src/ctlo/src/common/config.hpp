#ifndef CT_ICP_CONFIG_H
#define CT_ICP_CONFIG_H

#include <iostream>
#include <yaml-cpp/yaml.h>
#include "tools/tool_color_printf.hpp"

namespace
{
#define OPTION_CLAUSE(node_name, option_name, param_name, type)       \
     if (node_name[#param_name])                                      \
     {                                                                \
          option_name.param_name = node_name[#param_name].as<type>(); \
     }

     /* A Macro which finds an option in a YAML node */
#define FIND_OPTION(node_name, option_name, param_name, type)         \
     if (node_name[#param_name])                                      \
     {                                                                \
          option_name.param_name = node_name[#param_name].as<type>(); \
     }

     YAML::Node GetNode(const std::string &config_path)
     {
          try
          {
               return YAML::LoadFile(config_path);
          }
          catch (...)
          {
               std::cout << ANSI_COLOR_RED << "Could not load the file " << config_path << " from disk." << ANSI_COLOR_RESET << std::endl;
               throw;
          }
     }

}
#endif // CT_ICP_CONFIG_H
