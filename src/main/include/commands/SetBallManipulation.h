/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "subsystems/Feeder.h"
#include "subsystems/BallHolder.h"
#include "subsystems/FloorIntake.h"

class SetBallManipulation
    : public frc2::CommandHelper<frc2::ParallelCommandGroup,
                                 SetBallManipulation> {
 public:
  SetBallManipulation(Feeder *, BallHolder *, FloorIntake *, double floorIntake, double indexRoller, double MagazineBelts, double FeederMotor, bool intaking = false);
};
