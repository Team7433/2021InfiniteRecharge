/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Vision.h"
#include "subsystems/Arm.h"
#include "subsystems/Shooter.h"

#include <units/length.h>
#include <units/math.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DistanceSet
    : public frc2::CommandHelper<frc2::CommandBase, DistanceSet> {
 public:
  DistanceSet(Vision* vision, Arm* arm, Shooter* shooter);
  DistanceSet(units::meter_t distanceM, Arm* arm, Shooter* shoorter);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  Vision* m_vision;
  Arm* m_arm;
  Shooter* m_shooter;
  units::meter_t m_distanceM = 0.0_m;
  bool m_distanceOveride = false;


};
