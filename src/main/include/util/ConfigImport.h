// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctime>
#include <iostream>
#include <iomanip>

#include <frc/smartdashboard/SmartDashboard.h>

class ConfigImport {
  public:

    ConfigImport();
    
    static ConfigImport& GetInstance();

    bool LoadConfig(std::string fileLocation);

  private:
    std::vector<std::vector<double>> m_paramaters = {};
    std::vector<int> m_errorLines = {};

    std::string m_modifiedTime;

    std::string m_configFileName;
};
