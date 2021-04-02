/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Counter.h>

#include <networktables/NetworkTableInstance.h>

#include "subsystems/Arm.h"

#include <math.h>
#include <units/math.h>
#include <units/length.h>

#include "Constants.h"
using namespace VisionConstants;
class Vision : public frc2::SubsystemBase {
 public:
  Vision(Arm* arm);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  bool getPowerPortDetected();

  units::degree_t getPowerPortHorizontalAngle();
  
  units::degree_t getPowerPortVerticalAngle();

  double getCamMode();

  void changeCamMode(int mode);

  units::meter_t getPortDistance();

  units::meter_t getPortDistanceBumper();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  units::millimeter_t HC;
  units::millimeter_t HeightOfTarget_datam;
  units::degree_t PHI;
  units::degree_t AngleOfArm;
  Arm* m_arm;
};
