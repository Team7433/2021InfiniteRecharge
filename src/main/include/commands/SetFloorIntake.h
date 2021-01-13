/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/FloorIntake.h"

#include "Constants.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetFloorIntake
    : public frc2::CommandHelper<frc2::CommandBase, SetFloorIntake> {
  public:
    SetFloorIntake(FloorIntake * floorintake, FloorIntakeConstants::Position position, double rollerSpeed);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
  private:
    FloorIntake * m_intake;
    FloorIntakeConstants::Position m_position;
    double m_roller;
};
