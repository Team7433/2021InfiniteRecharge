/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"
#include <frc/DoubleSolenoid.h>
#include <Constants.h>

using namespace FloorIntakeConstants;

class FloorIntake : public frc2::SubsystemBase {
 public:
  FloorIntake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void Set(Position position, double roller);

 private:
  
  TalonSRX * m_RollerMotor = new TalonSRX{kIntakeMotorId};
  frc::DoubleSolenoid m_positionSolenoid{kSolonoidPortAId, kSolonoidPortBId};
};
