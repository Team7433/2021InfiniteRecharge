/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetBallManipulation.h"

#include "commands/SetFeeder.h"
#include "commands/SetBallHolder.h"
#include "commands/SetFloorIntake.h"
#include "commands/StoreBallHolder.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
SetBallManipulation::SetBallManipulation(Feeder * feeder, BallHolder * holder, FloorIntake * intake, double floorIntake, double indexRoller, double magazineBelts, double feederMotor, bool intakeing) {
  
  if (intakeing == false) {
    if (floorIntake != 0) {
      AddCommands(SetFeeder(feeder, BeltPosition::In, feederMotor),
                  SetBallHolder(holder, indexRoller, magazineBelts),
                  SetFloorIntake(intake, Position::Out, floorIntake)); 
    } else {
      AddCommands(SetFeeder(feeder, BeltPosition::In, feederMotor),
                  SetBallHolder(holder, indexRoller, magazineBelts),
                  SetFloorIntake(intake, Position::In, floorIntake)); 
    }
    
  } else {
    AddCommands(SetFeeder(feeder, BeltPosition::In, feederMotor),
    StoreBallHolder(holder, intake, indexRoller, magazineBelts, floorIntake));
  }
  
}
