// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <iostream>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/Timer.h>

#include "subsystems/FloorIntake.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/BallHolder.h"
#include "subsystems/Feeder.h"
#include "subsystems/Gyro.h"
#include "subsystems/Vision.h"
#include "subsystems/Arm.h"
#include "subsystems/AutoVaribles.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include "commands/DriveWithJoystick.h"
#include "commands/SetFloorIntake.h"
#include "commands/RunShooter.h"
#include "commands/SetBallHolder.h"
#include "commands/SetFeeder.h"
#include "commands/SetBallManipulation.h"
#include "commands/ChangeCamMode.h"
#include "commands/TurnToTarget.h"
#include "commands/SetArmAngle.h"
#include "commands/ManualArmControl.h"
#include "commands/DriveRunProfile.h"
#include "commands/UnloadMagazine.h"
#include "commands/AutoTarget.h"
#include "commands/DriveMotionControl.h"
class EightBallAutoA
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 EightBallAutoA> {
  public:
    EightBallAutoA(FloorIntake*, DriveTrain*, Shooter*, BallHolder*, Feeder*, Gyro*, Vision*, Arm*, AutoVaribles*);
    units::meter_t GetStartRightPos() {std::cout << "GetRightInside Function: " << m_rightStartPos << "\n"; return m_rightStartPos;}
    units::meter_t GetStartLeftPos() {return m_leftStartPos; }
    units::meter_t GetStartDistance() {return m_startingDistance; }

    void SetStartedRightPos(units::meter_t setPos) {m_rightStartPos = setPos;}
    void setStartedLeftPos(units::meter_t setPos) {m_leftStartPos = setPos;}

    double m_numberTest = 0.0;
    units::meter_t m_leftStartPos;
    units::meter_t m_rightStartPos = 6_m;
    units::meter_t m_startingDistance = 2_m;
    
  private: 
    // frc2::Timer m_timer;
    

    
   
  };
