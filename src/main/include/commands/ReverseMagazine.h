// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <functional>



#include "subsystems/BallHolder.h"
#include "subsystems/FloorIntake.h"
#include "subsystems/Feeder.h"
#include <frc/GenericHID.h>
#include <frc2/Timer.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ReverseMagazine
    : public frc2::CommandHelper<frc2::CommandBase, ReverseMagazine> {
 public:
  ReverseMagazine(BallHolder*, FloorIntake*, Feeder*, std::function<RobotContainerConstants::IntakeState()>, std::function<bool()> LeftTrig, std::function<bool()> RightTrig);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
 BallHolder* m_ballHolder;
 FloorIntake* m_floorIntake;
 Feeder* m_feeder;

 frc2::Timer m_timer;

 std::function<RobotContainerConstants::IntakeState()> m_intakeState;

 std::function <bool()> m_leftTrig;
 std::function <bool()> m_rightTrig;

 double m_startingMagazine;
 double m_startingFeeder;
 double m_startingIndexer;
 double m_startingIntake;

 bool m_lastSensorBeltIn = false;

 double m_intakeSpeed = 0.5;
 double m_indexRunSpeed = 0.3;
 double m_magazineRunSpeed = 0.3;

 FloorIntakeConstants::Position m_startingIntakePos;

};
