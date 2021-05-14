// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include <frc2/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/RGBStrip.h"
#include "subsystems/Shooter.h"
#include "subsystems/Arm.h"
#include "subsystems/Vision.h"
#include "subsystems/DriveTrain.h"

#include <stdlib.h>
#include <time.h>




/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class StatusLight
    : public frc2::CommandHelper<frc2::CommandBase, StatusLight> {
 public:
  StatusLight(RGBStrip*, Arm*, Shooter*, DriveTrain*, Vision*);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  RGBStrip* m_RGBStrip;
  Arm* m_arm;
  Shooter* m_shooter;
  Vision* m_vision;
  DriveTrain* m_driveTrain;
  frc2::Timer m_timer;

  double m_countR;
  double m_countG;
  double m_countB;
 
  double m_signR;
  double m_signG;
  double m_signB;
};
