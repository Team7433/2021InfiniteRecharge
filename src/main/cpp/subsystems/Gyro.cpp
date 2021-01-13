/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Gyro.h"
#include <frc/DriverStation.h>

Gyro::Gyro() {
    try {
       /* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
       /* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
       /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
        m_gyro = new AHRS(frc::SPI::Port::kMXP);

    } catch (std::exception ex ) {

        std::string err_string = "Error Instantiating navX-MXP: ";
        err_string += ex.what();
        frc::DriverStation::ReportError(err_string.c_str());
    }



    Reset();

}

// This method will be called once per scheduler run
void Gyro::Periodic() {
    frc::SmartDashboard::PutNumber("Gyro Yaw Value", GetYaw());
    frc::SmartDashboard::PutNumber("Gyro Roll Value", GetRoll());
    frc::SmartDashboard::PutNumber("Gyro Pitch Value", GetPitch());
}

double Gyro::GetYaw() {
    return m_gyro->GetYaw();
}

void Gyro::Reset() {
    return m_gyro->Reset();
}

double Gyro::GetPitch() {
    return m_gyro->GetPitch();
}

double Gyro::GetRoll() {
    return m_gyro->GetRoll();
}
