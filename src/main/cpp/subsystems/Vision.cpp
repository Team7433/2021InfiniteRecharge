/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Vision.h"


Vision::Vision(Arm* arm) {
    m_arm = arm;
    

}

// This method will be called once per scheduler run
void Vision::Periodic() {
    frc::SmartDashboard::PutBoolean("Vision/TargetDetected", getPowerPortDetected());
    frc::SmartDashboard::PutNumber("Vision/Target Distance Bumper", getPortDistanceBumper().to<double>());
    frc::SmartDashboard::PutNumber("Vision/Target Distance", getPortDistance().to<double>());
}

bool Vision::getPowerPortDetected() {
    if (nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0) == 1) {
        return true;
    }
    return false;
}

units::degree_t Vision::getPowerPortHorizontalAngle() {
    std::cout << nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0) << "LimeLightTX" << std::endl;
    return units::degree_t( nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0) );
}

units::degree_t Vision::getPowerPortVerticalAngle() {
    return units::degree_t( nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0) );
}

double Vision::getCamMode() {
    return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("camMode", 0);
}
void Vision::changeCamMode(int mode) {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", mode);
}

units::meter_t Vision::getPortDistance() {
    if (getPowerPortDetected() == true) {
        AngleOfArm = m_arm->GetArmAngleUnits();
        
        HC = klengthOfArm * units::math::sin( AngleOfArm + kangleOffsetCamera ) + kheightOfRobot;
        // frc::SmartDashboard::PutNumber("Vision/CameraHeight", HC.to<double>());
        HeightOfTarget_datam = kheightOfTarget - HC;
        // frc::SmartDashboard::PutNumber("Vision/", PHI.to<double>());
        return HeightOfTarget_datam / units::math::tan( AngleOfArm + kangleOfCamera + getPowerPortVerticalAngle() );
    } else {
        return -1_m;
    }
}

units::meter_t Vision::getPortDistanceBumper() {

    if (getPowerPortDetected() == true) {
        AngleOfArm = m_arm->GetArmAngleUnits();

        // return getPortDistance() + (klengthOfArm * cos(AngleOfArm+kangleOffsetCamera)*kPi/180);
        return units::millimeter_t(getPortDistance() + klengthOfArm * units::math::cos( AngleOfArm + kangleOffsetCamera ) - klengthPivotToBumper );
    } else {
        return -1_m;
    }


}
