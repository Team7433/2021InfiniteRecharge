/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix.h>

#include <frc/DigitalInput.h>

#include <Constants.h>

using namespace BallHolderConstants;

class BallHolder : public frc2::SubsystemBase {
 public:
  BallHolder();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void SetIndexer(double value);
  void SetMagazine(double value);

  bool GetSensorBeltIn();
  bool GetSensorBeltFull();
  bool GetSensorIndexerOut();
  bool GetSensorIndexerIn();
  bool GetSensorBeltMiddle();

 private:

    TalonSRX * m_indexerMotor = new TalonSRX{kIndexerMotorId};
    TalonSRX * m_magazineMotor = new TalonSRX{kMagazineMotorId};

    frc::DigitalInput m_sensor_beltIn{ksensorId_BeltIn};
    frc::DigitalInput m_sensor_beltFull{ksensorId_BeltFull};
    frc::DigitalInput m_sensor_IndexerOut{ksensorId_OutIndexer};
    frc::DigitalInput m_sensor_IndexerIn{ksensorId_InIndexer};
    frc::DigitalInput m_sensor_BeltMiddle{ksensorId_beltMiddle};

};
