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

  void SetFeeder(double value);
  void SetFeederVelocity(double velocity);
  void RunRevolutions(double revolutions, double maxAccel, double maxVelo);

 private:

  void configPID(double P, double I, double D, double Izone, double MaxAccumulator) {
    m_feederMotor->Config_kP(kPIDslotID, P, ktimeoutMs);
    m_feederMotor->Config_kI(kPIDslotID, I, ktimeoutMs);
    m_feederMotor->Config_kD(kPIDslotID, D, ktimeoutMs);
    m_feederMotor->Config_IntegralZone(kPIDslotID, Izone, ktimeoutMs);
    m_feederMotor->ConfigMaxIntegralAccumulator(kPIDslotID, MaxAccumulator, ktimeoutMs);

  }
  TalonSRX * m_feederMotor = new TalonSRX{kFeederMotorId};

};
