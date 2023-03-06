// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "RobotContainer.h"
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include "Constants.h"
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/RamseteController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc/Filesystem.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>
#include <wpi/fs.h>
#include <frc2/command/button/JoystickButton.h>
#include <photonlib/PhotonUtils.h>
#include <numbers>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "commands/PlacementSequence.h"
#include "commands/Balance.h"
#include <frc/Timer.h>

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

}

void RobotContainer::ConfigureButtonBindings() {
   m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        if(m_joystick.GetLeftStickButtonPressed())
        {
          if(m_drive.table->GetNumber("camMode", 0) == 0)
          {
            m_drive.table->PutNumber("camMode", 1);
          }
          else
          {
            m_drive.table->PutNumber("camMode", 0);
          }
        }
        if(m_joystick.GetRightStickButton())
        {
          m_drive.slewRateFlag = !m_drive.slewRateFlag;
        }
        double x = -m_joystick.GetRawAxis(1);
        double z = 0.0;
        if(m_joystick.GetAButton())
        {
          //photonlib::PhotonPipelineResult result = m_drive.camera.GetLatestResult();
          if(m_drive.table->GetNumber("tv", 0.0))
          {
            if (m_drive.table->GetNumber("tx", 0.0) < -3.0)
            {
              z = 0.37;
            }
            if (m_drive.table->GetNumber("tx", 0.0) > 3.0)
            {
              z = -0.37;
            }
            //z = controller.Calculate((result.GetBestTarget().GetYaw()), 0);
          }
        }
        else 
        {
          z = (m_joystick.GetRawAxis(2) - m_joystick.GetRawAxis(3)) / 2.0;
        }
        m_drive.ArcadeDrive(x, z);
      },
      {&m_drive}));

  m_arm.SetDefaultCommand(frc2::RunCommand
  {
    [this]
    {
      double x = -m_joystick.GetRawAxis(5);
      m_arm.moveArm(1.2_V * x);

      if(m_joystick.GetBackButton())
      {
        m_arm.moveIntake(-12_V);
      }
      else if(m_joystick.GetStartButton())
      {
        m_arm.moveIntake(12_V);
      }
      else m_arm.moveIntake(0_V);
    },
    {&m_arm}
  });

  m_joystick.B().OnTrue(Balance(&m_drive).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {

  printf("Kelvin1 %.03f\n", frc::Timer::GetFPGATimestamp().value());

  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint
  {
    frc::SimpleMotorFeedforward<units::meters>
    {
      testRobot::kS, testRobot::kV, testRobot::kA
    },
    frc::DifferentialDriveKinematics(testRobot::kTrackWidth), 2_V
  };
  
  frc::TrajectoryConfig config(0.5_mps, 0.4_mps_sq);
  config.SetKinematics(frc::DifferentialDriveKinematics(testRobot::kTrackWidth));
  config.AddConstraint(autoVoltageConstraint);
  frc::Pose2d initialPose = frc::Pose2d{0_m, 0_m, 0_deg};
  frc::Pose2d finalPose;

  
  finalPose = frc::Pose2d{2_m,2_m,0_deg};

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    initialPose,
    {/*frc::Translation2d{2_m,0_m}, frc::Translation2d{1_m,1_m}*/},
    finalPose,
    config
  );
  frc::Trajectory pathWeaverTraj;
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "output" / "path3.wpilib.json";
  pathWeaverTraj = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  //m_drive.m_field.GetObject("traj")->SetTrajectory(trajectory);
  
  m_drive.resetOdometry(trajectory.InitialPose());
  frc2::RamseteCommand ramseteCommand
  {
    trajectory,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{testRobot::kS, testRobot::kV, testRobot::kA},
    frc::DifferentialDriveKinematics(testRobot::kTrackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{.5, 0, 0},
    frc2::PIDController{.5, 0, 0},
    [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
    {&m_drive},
  };


  printf("Kelvin2 %.03f\n", frc::Timer::GetFPGATimestamp().value());
  double armPos = m_arm.getEncoderValue();
  return 
      std::move(ramseteCommand).FinallyDo([this] (bool end){m_drive.tankDriveVolts(0_V,0_V);})
      .AndThen([this] {m_arm.moveArm(2_V); }, {&m_arm}).Until([this] {return m_arm.getLimitSwitch();})
      .AndThen([this] {m_arm.moveIntake(-2_V);}, {&m_arm}).WithTimeout(2_s)
      .AndThen([this] { m_arm.moveArm(-2_V);}, {&m_arm}).Until([this, armPos] {return m_arm.getLimitSwitch() <= armPos;})
      .AndThen([this] {m_drive.ArcadeDrive(0, .3);}, {&m_drive}).Until([this] {return m_drive.getPose().Rotation().Degrees().value() == 180.0;});
      //lmoo 
}

frc2::CommandPtr RobotContainer::Autonomous2() {
  frc::Trajectory pathWeaverTraj;
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "output" / "test.wpilib.json";
  pathWeaverTraj = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  //m_drive.m_field.GetObject("traj")->SetTrajectory(trajectory);
  
  m_drive.resetOdometry(pathWeaverTraj.InitialPose());
  frc2::RamseteCommand ramseteCommand
  {
    pathWeaverTraj,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{testRobot::kS, testRobot::kV, testRobot::kA},
    frc::DifferentialDriveKinematics(testRobot::kTrackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{.5, 0, 0},
    frc2::PIDController{.5, 0, 0},
    [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
    {&m_drive},
  };
  return std::move(ramseteCommand).AndThen([this] {m_drive.tankDriveVolts(0_V,0_V);}, {&m_drive});
}

frc2::CommandPtr RobotContainer::AprilTagTrajectory() {
  
  printf("KelvinDU1 %.03f\n", frc::Timer::GetFPGATimestamp().value());
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint
  {
    frc::SimpleMotorFeedforward<units::meters>
    {
      testRobot::kS, testRobot::kV, testRobot::kA
    },
    frc::DifferentialDriveKinematics(testRobot::kTrackWidth), 2_V
  };
  
  frc::TrajectoryConfig config(0.5_mps, 0.4_mps_sq);
  config.SetKinematics(frc::DifferentialDriveKinematics(testRobot::kTrackWidth));
  config.AddConstraint(autoVoltageConstraint);
  frc::Pose2d initialPose = m_drive.getPose();
  frc::Pose2d finalPose;
  
  
  printf("KelvinDU2 %.03f\n", frc::Timer::GetFPGATimestamp().value());
  if(m_drive.table->GetNumber("tv", 0.0))
  {
    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
      vision::cameraHeight, vision::targetHeight, vision::cameraPitch, units::degree_t{m_drive.table->GetNumber("ty", 0.0)}
    );
    frc::Translation2d translation(range, frc::Rotation2d{units::degree_t{-m_drive.table->GetNumber("tx", 0.0)}});
    frc::Transform2d transform(translation, frc::Rotation2d{0_deg});
    finalPose = initialPose.TransformBy(transform);
    
  }
  else finalPose = frc::Pose2d{2_m,4_m,0_deg};
  
  printf("KelvinDU3 %.03f\n", frc::Timer::GetFPGATimestamp().value());

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    initialPose,
    {/*frc::Translation2d{2_m,0_m}, frc::Translation2d{1_m,1_m}*/},
    finalPose,
    config
  );
  

  printf("KelvinDU4 %.03f\n", frc::Timer::GetFPGATimestamp().value());

  frc2::RamseteCommand ramseteCommand
  {
    trajectory,
    [this]() {return m_drive.getPose();},
    frc::RamseteController{},
    frc::SimpleMotorFeedforward<units::meters>{testRobot::kS, testRobot::kV, testRobot::kA},
    frc::DifferentialDriveKinematics(testRobot::kTrackWidth),
    [this]() {return m_drive.getWheelSpeed();},
    frc2::PIDController{.5, 0, 0},
    frc2::PIDController{.5, 0, 0},
    [this](auto left, auto right){m_drive.tankDriveVolts(left, right);},
    {&m_drive},
  };
printf("KelvinDU5\n");
  return frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.tankDriveVolts(0_V, 0_V); }, {})).ToPtr();
      //lmoo
}