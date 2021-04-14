// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <units/length.h>
#include <units/angle.h>

class AutoVaribles : public frc2::SubsystemBase {
 public:
  AutoVaribles();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  units::meter_t a_leftStartPos = 0.0_m;
  units::meter_t a_rightStartPos = 0.0_m;  
  units::degree_t a_targetAngle = 0.0_deg;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
