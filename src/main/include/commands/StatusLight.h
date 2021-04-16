// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include <frc2/Timer.h>

#include <subsystems/RGBStrip.h>
#include <subsystems/Arm.h>
#include <subsystems/Vision.h>


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
  StatusLight(RGBStrip*, std::function<units::degree_t()> targetAngle, std::function<units::degree_t()> currentAngle, std::function<bool()> targetDeteced, std::function<double()> targetVelocity);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  RGBStrip* m_RGBStrip;
  std::function<units::degree_t()> m_targetAngle;
  std::function<units::degree_t()> m_currentAngle;
  std::function<double()> m_targetVelocity;
  std::function<bool()> m_targetDetected;

  frc2::Timer m_timer;
};
