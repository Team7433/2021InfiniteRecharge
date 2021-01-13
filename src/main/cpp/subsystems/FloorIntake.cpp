/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/FloorIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>

FloorIntake::FloorIntake() {
    m_RollerMotor->ConfigFactoryDefault();
}

// This method will be called once per scheduler run
void FloorIntake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake/Current", m_RollerMotor->GetOutputCurrent());
    
}


void FloorIntake::Set(Position position, double roller) {
    m_RollerMotor->Set(ControlMode::PercentOutput, roller);
    
    //Change the following in constants file
    if (position == FloorIntakeConstants::Position::Out) {
        m_positionSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
        frc::SmartDashboard::PutBoolean("Intake Arm", true);
    } else {
        m_positionSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
        frc::SmartDashboard::PutBoolean("Intake Arm", false);
    }
}