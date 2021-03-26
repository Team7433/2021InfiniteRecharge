/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Arm.h"
#include <units/length.h>
#include <units/angle.h>

Arm::Arm() {

    m_armMotor->ConfigFactoryDefault();


    m_armMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, Ktimeout);
    m_armEncoder->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, Ktimeout);


    m_armMotor->SetNeutralMode(NeutralMode::Brake);

    m_armMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyClosed, Ktimeout);
    m_armMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, Ktimeout);
    
    m_armMotor->SetSensorPhase(true);
    m_armEncoder->SetSensorPhase(true);

    m_armMotor->ConfigNeutralDeadband(0);
    

    // configures motors PID
    m_armMotor->Config_kF(0, Kf, Ktimeout);
    m_armMotor->Config_kP(0, Kp, Ktimeout);
    m_armMotor->Config_kI(0, Ki, Ktimeout);
    m_armMotor->Config_kD(0, Kd, Ktimeout);
    m_armMotor->ConfigMotionSCurveStrength(4, Ktimeout);

    m_armMotor->ConfigMotionAcceleration(KmotionAcceleration, Ktimeout);
    m_armMotor->ConfigMotionCruiseVelocity(KmotionCruiseVelocity, Ktimeout);  

    m_armMotor->ConfigClearPositionOnLimitR(true, Ktimeout);

}

// This method will be called once per scheduler run
void Arm::Periodic() {
    frc::SmartDashboard::PutNumber("Arm velocity", GetVelocity());
    frc::SmartDashboard::PutNumber("Arm Position", GetPosition());
    frc::SmartDashboard::PutNumber("Arm Output power", GetMotorOutput());
    //frc::SmartDashboard::PutNumber("Arm Error", GetError());
    //frc::SmartDashboard::PutNumber("Arm target position", GetTargetPosition());
    frc::SmartDashboard::PutNumber("Arm Angle: ", GetArmAngle());
    frc::SmartDashboard::PutNumber("Motor Arm Angle: ", GetArmAngleMotor());
    frc::SmartDashboard::PutNumber("Arm Angle Difference ", (GetArmAngle() - GetArmAngleMotor()));
}

void Arm::ManualControl(double Output) {
    m_armMotor->Set(ControlMode::PercentOutput, Output);
}

void Arm::SetPosition(double Position) {
    m_armMotor->Set(ControlMode::MotionMagic, Position);
}

void Arm::SetAngle(units::angle::armEncoderUnits_t Angle) {
    m_armMotor->Set(ControlMode::MotionMagic, Angle.to<double>());
}

void Arm::SetAngle(double Angle) {
    //some complex math equations ++--++--
    m_armMotor->Set(ControlMode::MotionMagic, (53.8) * (Angle - armAngleOffset));
}

void Arm::SetLock(Lock_Position lock_position) {

    if (lock_position == Lock_Position::Lock) {
        m_lockSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        m_lockSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

double Arm::GetPosition() {
    return m_armMotor->GetSelectedSensorPosition();
}

double Arm::GetVelocity() {
    return m_armMotor->GetSelectedSensorVelocity();
}
double Arm::GetMotorOutput() {
    return m_armMotor->GetMotorOutputPercent();
}
double Arm::GetError() {
    return m_armMotor->GetClosedLoopError();
}
double Arm::GetTargetPosition() {
    return m_armMotor->GetClosedLoopTarget();
}
double Arm::GetArmAngle() {
    // return (m_armMotor->GetSelectedSensorPosition() / 53.8) +  6;
    // return 360*(m_armEncoder->GetSelectedSensorPosition()/4096);
    // double value = (m_armEncoder->GetSelectedSensorPosition() / 4096.0);
    // printf("%f \n", value);
    return ((m_armEncoder->GetSelectedSensorPosition() / 4096.0) * 360 ) + 196;
}

double Arm::GetArmAngleMotor() {
   
    return (m_armMotor->GetSelectedSensorPosition() / 53.8) +  armAngleOffset;

}

// units::degree_t Arm::CalculateAngleFromDistance(units::meter_t distance) {

//     double distanceMM = distance.to<double>();

//     return units::degree_t(15.5504 + (130.439 / (distanceMM) + 2.46224)));

// }

