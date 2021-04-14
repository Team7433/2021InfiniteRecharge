// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/Timer.h>

#include "subsystems/BallHolder.h"
#include "subsystems/Feeder.h"
#include "SetFloorIntake.h"
#include "Constants.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class UnloadMagazine
    : public frc2::CommandHelper<frc2::CommandBase, UnloadMagazine> {
 public:
  UnloadMagazine(BallHolder*, Feeder*, FloorIntake*,bool intakePosition = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  BallHolder * m_ballHolder;
  Feeder * m_feeder;
  FloorIntake * m_intake;
  frc2::Timer m_timer;

  bool m_intakePos;
  bool m_done = false;

};
