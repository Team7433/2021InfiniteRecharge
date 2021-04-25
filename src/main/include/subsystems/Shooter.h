/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

#include <ctre/Phoenix.h>

using namespace ShooterConstants;

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void ManualControl(double Output);

  void SetVelocity(double Velocity);

  void configPID(double P, double I, double D, double Izone, double MaxAccumulator);

  double GetVelocity();

  double GetVelocityLoopError();

  double GetVelocityLoopTarget();

  double GetIAccumulator();

  double GetPercentageOutput();

  void configKF(double kF);

  void setState(idleState state) { m_idleState = state; }

  idleState getState() const { return m_idleState; }


  void Periodic();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX * m_shooterA = new WPI_TalonFX{kShooterAID};
  WPI_TalonFX * m_shooterB = new WPI_TalonFX{kShooterBID};

  idleState m_idleState = idleState::targetReached;
  std::string idleStateTypes[2] = {"Target Reached", "Reaching Target"};

};
