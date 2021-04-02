
#include "util/MotionProfileRunner.h"
#include <iostream>
#include <fstream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

using namespace Iona;

bool MotionProfileRunner::LoadFromFile(std::string newPath) {

    if (newPath == m_loadedPathName && m_profileStarted == false) {
    std::cout << "Path Allready Loaded: " << newPath << "\n";
    return true;
    }

    m_loadedPathName = newPath;

    // Start notifier running periodictask
    m_notifier.StartPeriodic(0.005_s);

    TrajectoryPoint pointLeft;
    TrajectoryPoint pointRight;

      //Check if we underrun then reset it if it has so we don't have that run over into the new profile
    if (_statusLeft.hasUnderrun || _statusRight.hasUnderrun)  {

        m_leftMotor->ClearMotionProfileHasUnderrun(kTimeoutMs);
        m_rightMotor->ClearMotionProfileHasUnderrun(kTimeoutMs);
    }

      //Clear Trajectorys that might have acidentally stayed in the talon
    m_leftMotor->ClearMotionProfileTrajectories();
    m_rightMotor->ClearMotionProfileTrajectories();

    //Sets Base trajectory Period
    m_leftMotor->ConfigMotionProfileTrajectoryPeriod(0, kTimeoutMs);
    m_rightMotor->ConfigMotionProfileTrajectoryPeriod(0, kTimeoutMs);

    //Reading Profile From File

    //Finding profile length
    int profileLength = 0;

    std::string line;
    std::ifstream profileforlength(m_pathToProfileFolder + m_loadedPathName + ".csv");
    if (profileforlength.is_open()) {
        //std::cout << "Found File" << "\n";
        while (getline(profileforlength, line)) {
           profileLength++;
        }
        printf("Profile Length: %i \n", profileLength);
        profileforlength.close();
    }
    std::ifstream myfile (m_pathToProfileFolder + m_loadedPathName + ".csv");
    if (myfile.is_open()) {
        int i = 0;//Stores the current Profile id
        while (getline (myfile, line)) {        //Repeat through each line in the code and set the varible line to the current line
        const int dataPoints = 7;             //The Ammout of data points in a line
        double lineData [dataPoints];         //Array with the data points in it
        //bool endOfLine = false;             //I don't think this is used so i commened it out
        std::string shortenedLine = line;           //Set the String that we will iterate though to the line from the file
        //frc::SmartDashboard::PutNumber("MotionProfilePoints", i);
            for (int c =0; c < dataPoints; c++) { //Repeat though all of the data points in the line and save them to the array
                //Find the first Comma (This is after the current data point)
                int comma_first = shortenedLine.find(',');

                //Using the point in the sting the comma is at get a sub string of the current data point in string form
                std::string dataPoint_str = shortenedLine.substr(0, comma_first);

                //Convert the sting to a double
                double dataPoint = std::stod(dataPoint_str);

                //Add this to the array corosponding to where it was in the line
                lineData [c] = dataPoint;

                //Shorten the string taking out the data point we just got and the comma
                shortenedLine = shortenedLine.substr(comma_first+1, shortenedLine.length()-comma_first-1);
            }; //For to seperate commars

            //Grab data from the array
        double positionRotLeft  = lineData[0];
        double VelocityRPMLeft  = lineData[1];
        double positionRotRight  = lineData[2];
        double VelocityRPMRight  = lineData[3];

        //Print Where we are up to in the sending (This could probably be at the top of the loop)
        printf("Sending point %i of %i\n", i, profileLength);
        frc::SmartDashboard::PutNumber("Drive/MotionProfiling/Sent Point", i);

        //For each point, fill our structure and pass it to API
        pointLeft.position = positionRotLeft * m_metersToSensorUnits; //Covert Revolutions to Units
        pointLeft.velocity = VelocityRPMLeft * m_mpsToUnitsPer100ms; //Covert RPM to Units/100ms
        pointLeft.headingDeg = 0; //Furture feature Not Used
        pointLeft.profileSlotSelect0 = 0; //Idk
        pointLeft.profileSlotSelect1 = 0; //Furture feature Not Used
        pointLeft.timeDur = 20; //Set how long you want the talon to stay on this point in ms
        pointLeft.zeroPos = false;
        pointLeft.isLastPoint = false;

        pointRight.position = positionRotRight * m_metersToSensorUnits; //Covert Revolutions to Units
        pointRight.velocity = VelocityRPMRight * m_mpsToUnitsPer100ms; //Covert RPM to Units/100ms
        pointRight.headingDeg = 0; //Furture feature Not Used
        pointRight.profileSlotSelect0 = 0; //Idk
        pointRight.profileSlotSelect1 = 0; //Furture feature Not Used
        pointRight.timeDur = 20; //Set how long you want the talon to stay on this point in ms
        pointRight.zeroPos = false;
        pointRight.isLastPoint = false;

        if (i == 0) {
            //Set this true only on the first point
            pointLeft.zeroPos = true; 
            pointRight.zeroPos = true; 
        }

        //if last point set last point to true
        if ((i+1) == profileLength) {
            pointLeft.isLastPoint = true;
            pointRight.isLastPoint = true;
            //printf("Is Last Point \n");
        }

        //Set The Points to the talon
        m_leftMotor->PushMotionProfileTrajectory(pointLeft);
        m_rightMotor->PushMotionProfileTrajectory(pointRight);

        i++;
        }
    myfile.close();
    m_profileLength = profileLength;
    m_profileLoaded = true;
    m_profileStarted = false;
    return false;

    } else {
        return true;
    }
}

void MotionProfileRunner::Start(int startPoints) {
  if (m_profileLoaded) {
    m_startPoints = startPoints;
    m_running = true;
    m_state = 0;
    m_leftMotor->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
    m_rightMotor->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
    m_LoggingFile = new std::ofstream (m_pathToProfileFolder + "/logs/" + m_loadedPathName + "log.csv");
    
    if (m_LoggingFile->is_open()) {

      *m_LoggingFile << "Pos,velocity,target pos,traj vel,traj pos,error,%output,Pos,velocity,target pos,traj vel,traj pos,error,%output" << std::endl;

    }

  }
}

void MotionProfileRunner::Stop() {
  m_running = false;
  m_state = 3;
  m_notifier.Stop();
  m_profileLoaded = false;
  m_leftMotor->ClearMotionProfileTrajectories();
  m_rightMotor->ClearMotionProfileTrajectories();
}

void MotionProfileRunner::UpdateRunner() {
  //Make Sure we never get stuck
  if (_loopTimeout < 0) {
    //All Good do nothing
  } else {
    if (_loopTimeout == 0) {
      //Something is wrong
    } else {
      --_loopTimeout;
    }
  }

  if (m_LoggingFile->is_open()) {

    *m_LoggingFile << m_leftMotor->GetSelectedSensorPosition() << "," << m_leftMotor->GetSelectedSensorVelocity() << "," << m_leftMotor->GetClosedLoopTarget() << "," << m_leftMotor->GetActiveTrajectoryVelocity() << "," << m_rightMotor->GetActiveTrajectoryPosition() << "," << m_leftMotor->GetClosedLoopError() << "," << m_leftMotor->GetMotorOutputPercent() << ",";
    *m_LoggingFile << m_rightMotor->GetSelectedSensorPosition() << "," << m_rightMotor->GetSelectedSensorVelocity() << "," << m_rightMotor->GetClosedLoopTarget() << ","  << m_rightMotor->GetActiveTrajectoryVelocity() << "," << m_rightMotor->GetActiveTrajectoryPosition() << "," << m_rightMotor->GetClosedLoopError() << "," << m_rightMotor->GetMotorOutputPercent() << std::endl;

  }



  std::cout << "State: " << m_state << "\n";

  //We are in motion profile control mode
    switch (m_state) {
        case 0:
        //Waiting for enough points in talon to start the profile

        //Checks if the buffer count from the end of last run is enough to start the profile
        if((_statusLeft.btmBufferCnt > m_startPoints) && (_statusRight.btmBufferCnt > m_startPoints)) {
            printf("Start Moving");
            //Start (once) the motion profile 
            m_leftMotor->Set(ControlMode::MotionProfile, SetValueMotionProfile::Enable);
            m_rightMotor->Set(ControlMode::MotionProfile, SetValueMotionProfile::Enable);

            m_state = 1; //Set The State to Continue To the next phase
            _loopTimeout =  KNumLoopsTimeout; //Sets a amount of loops for the profiler to timeout
        }
        break;
        case 1:
        //This is called while the profile is being run
        //printf("BtmBufferCount: %i, TopBufferCnt %i \n", _statusLeft.btmBufferCnt, _statusLeft.topBufferCnt);
        if ((_statusLeft.isUnderrun == false) | (_statusRight.isUnderrun == false)) {
            
            _loopTimeout = KNumLoopsTimeout; //Sets a amount of loops for the profiler to timeout

        } else {
            // loop Timeout has not been reset and would have been decreased
            printf("Is Underrun \n");
        }

        if (_statusLeft.activePointValid &&_statusLeft.isLast)  {
            m_leftMotor->Set(ControlMode::MotionProfile, SetValueMotionProfile::Hold);
            m_rightMotor->Set(ControlMode::MotionProfile, SetValueMotionProfile::Hold);
            m_state = 3;
            _loopTimeout = -1;
            m_notifier.Stop();
            m_LoggingFile->close();
            m_loadedPathName = "null";
        }
        break;
    }
      m_leftMotor->GetMotionProfileStatus(_statusLeft);
      m_rightMotor->GetMotionProfileStatus(_statusRight);

  }
  void MotionProfileRunner::PeriodicTask() {
  m_leftMotor->ProcessMotionProfileBuffer();
  m_rightMotor->ProcessMotionProfileBuffer();
}



