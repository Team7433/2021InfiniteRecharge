/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/BallHolder.h"
#include "subsystems/FloorIntake.h"
#include <frc2/Timer.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class StoreBallHolder
    : public frc2::CommandHelper<frc2::CommandBase, StoreBallHolder> {
  public:
    StoreBallHolder(BallHolder *, FloorIntake * , double indexSpeed, double magazineSpeed, double intakeSpeed);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
  private:
    BallHolder *  m_ballholder;
    FloorIntake * m_intake;

    frc2::Timer m_timer = frc2::Timer();

    double m_magazineRunSpeed;
    double m_indexRunSpeed;
    double m_intakeSpeed;
    bool m_lastSensorBeltIn = false;
};
