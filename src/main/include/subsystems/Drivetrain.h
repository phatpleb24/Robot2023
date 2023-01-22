// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/drive/DifferentialDrive.h"
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include "ctre/Phoenix.h"
#include <units/length.h>
#include <numbers>
#include <frc/smartdashboard/Field2d.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <Constants.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/plant/DCMotor.h>

class Drivetrain : public frc2::SubsystemBase {
 public:
  
  static constexpr int32_t kUnitsPerRevolution = 2048;
  static constexpr double kGearRatio = 8;
  static constexpr units::inch_t kWhellRadiusInches = 2_in;
  static constexpr int k100msPerSecond = 10;
  frc::Field2d m_field;
  Drivetrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void ArcadeDrive(double xAxisSpeed, double l1, double r1);

  void resetOdometry(frc::Pose2d pos);

  frc::Pose2d getPose();
  frc::Pose2d alterPos(frc::Pose2d currentPose, units::meter_t x, units::meter_t y, units::degree_t deg);
  frc::Translation2d alterTrans(frc::Pose2d currentPose, units::meter_t x, units::meter_t y);
  frc::DifferentialDriveWheelSpeeds getWheelSpeed();

  void tankDriveVolts(units::volt_t left, units::volt_t right);

  void SetMaxOutput(double x);

 private:
  void Init();
  void UpdateOdometry();
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonFX m_rightFrontMotor{1};
  WPI_TalonFX m_rightFollowerMotor{2};
  WPI_TalonFX m_leftFrontMotor{3};
  WPI_TalonFX m_leftFollowerMotor{4};
  WPI_Pigeon2 m_imu{8};

//  static constexpr  c = kWhellRadiusInches * 2 * wpi::numbers::pi;
  //auto circumferenceToGear = c * kGearRatio / kUnitsPerRevolution;

  frc::DifferentialDrive diffDrive{m_leftFrontMotor, m_rightFrontMotor};
  frc::DifferentialDriveOdometry m_odometry{m_imu.GetRotation2d(),0_m,0_m};


  TalonFXSimCollection m_leftMasterSim {m_leftFrontMotor.GetSimCollection()};
  TalonFXSimCollection m_rightMasterSim {m_rightFrontMotor.GetSimCollection()};
  BasePigeonSimCollection m_pidgeonSim {m_imu.GetSimCollection()};

  frc::sim::DifferentialDrivetrainSim m_drivetrainSim
  {
    frc::LinearSystemId::IdentifyDrivetrainSystem(DriveConstants::kV,DriveConstants::kA,DriveConstants::kVAngular,DriveConstants::kAAngular),
    DriveConstants::trackWidth,
    frc::DCMotor::Falcon500(2),
    kGearRatio,
    kWhellRadiusInches,
    {0,0,0,0,0,0,0}
  };
  int DistanceToNativeUnits(units::meter_t position);
  int VelocityToNativeUnits(units::meters_per_second_t velocity);
  units::meters_per_second_t NativeUnitstoVelocityMPS(double sensorCounts);
  units::meter_t NativeUnitsToDistanceMeters(double sensorCounts);
};
