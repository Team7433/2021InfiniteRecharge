// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <ctre/phoenix.h>

class RGBStrip : public frc2::SubsystemBase {
 public:
  RGBStrip();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetRGBStrip(double R, double G, double B);
  void Rainbow();
  
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  CANifier* m_strip = new CANifier{16};
  double m_R;
  double m_G;
  double m_B;
 
  double m_countR;
  double m_countG;
  double m_countB;
 
  double m_signR;
  double m_signG;
  double m_signB;

};
