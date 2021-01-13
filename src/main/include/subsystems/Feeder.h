/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>


#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include "Constants.h"

using namespace FeederConstants;

class Feeder : public frc2::SubsystemBase {
 public:
  Feeder();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void SetPosition(BeltPosition position);

  void SetFeeder(double value);

 private:
  TalonSRX * m_feederMotor = new TalonSRX{kFeederMotorId};
  frc::DoubleSolenoid m_engagerSolonoid{kFeederSolonoidPortAId, kFeederSolonoidPortBId};
  
};
