/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/drive/DifferentialDrive.h>
#include "Constants.h"


using namespace DriveTrainConstants;

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void ArcadeDrive(double forward, double rotation, bool squaredInput);

  void CurvatureDrive(double forward, double rotation, bool quickTurn);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.


  WPI_TalonFX * m_leftDrive1 = new WPI_TalonFX{kLeftDrive1ID};
  WPI_TalonFX * m_leftDrive2 = new WPI_TalonFX{kLeftDrive2ID};
  WPI_TalonFX * m_rightDrive1 = new WPI_TalonFX{kRightDrive1ID};
  WPI_TalonFX * m_rightDrive2 = new WPI_TalonFX{kRightDrive2ID};

  frc::DifferentialDrive m_robotDrive{*m_leftDrive1, *m_rightDrive1};
  


};
