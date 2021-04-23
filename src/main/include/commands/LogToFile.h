// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <fstream>
#include <iostream>
#include <frc2/Timer.h>

#include "subsystems/DriveTrain.h"
#include "subsystems/Gyro.h"
// #include "subsystems/Vision.h"


class LogToFile
    : public frc2::CommandHelper<frc2::CommandBase, LogToFile> {
  public:
    LogToFile(DriveTrain *, Gyro *);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
  private:
    DriveTrain * m_driveTrain;
    Gyro * m_gyro;
    // std::string m_fileName;
    std::string m_logText;

};
