/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/length.h>
#include <units/velocity.h>

#include "Constants.h"

#include "util/MotionProfileRunner.h"


using namespace DriveTrainConstants;

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  void ArcadeDrive(double forward, double rotation, bool squaredInput);

  void CurvatureDrive(double forward, double rotation, bool quickTurn);
  // Set Motors To Break mode
  void SetBrakeMode() {
    m_rightDriveMaster->SetNeutralMode(NeutralMode::Brake);
    m_rightDriveSlave->SetNeutralMode(NeutralMode::Brake);
    m_leftDriveMaster->SetNeutralMode(NeutralMode::Brake);
    m_leftDriveSlave->SetNeutralMode(NeutralMode::Brake);
  }
  // Set Motors To Coast Mode
  void SetCoastMode() {
    m_rightDriveMaster->SetNeutralMode(NeutralMode::Coast);
    m_rightDriveSlave->SetNeutralMode(NeutralMode::Coast);
    m_leftDriveMaster->SetNeutralMode(NeutralMode::Coast);
    m_leftDriveSlave->SetNeutralMode(NeutralMode::Coast);
    }

  void SetSlot(int slot) {

    m_rightDriveMaster->SelectProfileSlot(slot, 0);
    m_leftDriveMaster->SelectProfileSlot(slot, 0);

  }

  bool SongFinished() {return m_orchestra->IsPlaying();}

  double getLeftEncoder() { return m_leftDriveMaster->GetSelectedSensorPosition(); }
  double getRightEncoder() { return m_rightDriveMaster->GetSelectedSensorPosition(); }
  
  units::meter_t getLeftDistance();
  units::meter_t getRightDistance();

  units::meters_per_second_t getLeftVelocity();
  units::meters_per_second_t getRightVelocity();

  void setVelocity(units::meters_per_second_t left, units::meters_per_second_t right);

  void MPStart() { m_profiler->Start(kMPStartBuffer); }

  void MPUpdate() { m_profiler->UpdateRunner(); }

  void MPStop() { m_profiler->Stop(); }

  bool MPIsFinished() { return m_profiler->IsFinishedProfile(); }

  bool MPLoad(std::string path) {return m_profiler->LoadFromFile(path); }

  void playSong() {m_orchestra->Play();}

  void setState(idleState state) { m_idleState = state; }

  void SstVelocityRamping(double ramp) {m_leftDriveMaster->ConfigOpenloopRamp(ramp, ktimeoutMs); m_rightDriveMaster->ConfigOpenloopRamp(ramp, ktimeoutMs);}

  idleState getState() const { return m_idleState; }


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  Orchestra* m_orchestra = new Orchestra();

  WPI_TalonFX * m_leftDriveMaster = new WPI_TalonFX{kLeftDrive1ID};
  WPI_TalonFX * m_leftDriveSlave = new WPI_TalonFX{kLeftDrive2ID};
  WPI_TalonFX * m_rightDriveMaster = new WPI_TalonFX{kRightDrive1ID};
  WPI_TalonFX * m_rightDriveSlave = new WPI_TalonFX{kRightDrive2ID};

  

  frc::DifferentialDrive m_robotDrive{*m_leftDriveMaster, *m_rightDriveMaster};
  
  Iona::MotionProfileRunner *m_profiler = new Iona::MotionProfileRunner(m_leftDriveMaster, m_rightDriveMaster);
  idleState m_idleState = idleState::targetReached;
  std::string idleStateTypes[2] = {"Target Reached", "Reaching Target"};

};
