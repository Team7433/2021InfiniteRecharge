#pragma once

#include <ctre/Phoenix.h>
#include <string.h>
#include <frc/Notifier.h>

namespace Iona {

    class MotionProfileRunner {
    
    public:

        /**
         * Creates A Motion Profile Runner For A Drivetrain
         * 
         * @param
         */
        MotionProfileRunner(WPI_TalonFX * leftMotor, WPI_TalonFX * rightMotor) : m_leftMotor(leftMotor), m_rightMotor(rightMotor), m_notifier(&MotionProfileRunner::PeriodicTask, this) {} 
        ~MotionProfileRunner() {}
        
         /**
          * Loads a profile from a file in the Profiles folder
          * 
          * @param newPath The name of the file and path to be loaded
          * @return If there was a issue with getting or interpreting the profile file
          */
        bool LoadFromFile(std::string);

        std::string GetLoadedPath() { return m_loadedPathName; }

        void Start(int);

        void Stop();

        void UpdateRunner();

        bool IsFinishedProfile() { return (m_state == 3); }

        int GetCurrentPoint() {  
          if (m_running) {
            return m_profileLength - _statusLeft.btmBufferCnt;
          }
          return 0;
        }

        void SetMetersToUnits(double value) { m_metersToSensorUnits = value; }

        void SetmpsToUnit100ms(double value) { m_mpsToUnitsPer100ms = value; }




    
    private:
      void PeriodicTask();

    WPI_TalonFX * m_leftMotor;
    WPI_TalonFX * m_rightMotor;

    bool m_profileLoaded = false;
    bool m_profileStarted = false;
    int m_profileLength;
    int m_startPoints;

    std::string m_loadedPathName;
    std::string m_pathToProfileFolder = "home/lvuser/Profiles/"; //Stores the path to the profile

    bool m_running;
    int m_state;

    MotionProfileStatus _statusLeft, _statusRight;

    int KNumLoopsTimeout = 10;
    int _loopTimeout = 0;
    int kTimeoutMs = 10, kPIDLoopIdx = 0, kSlotIdx = 0;

    double m_metersToSensorUnits = 4096;
    double m_mpsToUnitsPer100ms = 4096 / 60;

    frc::Notifier m_notifier;

    std::ofstream * m_LoggingFile;

    };

}