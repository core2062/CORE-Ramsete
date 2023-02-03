// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include "config.h"

#include <AHRS.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/GroupMotorControllers.h>
#include <units/angular_velocity.h>
#include <units/length.h>

using namespace frc;

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() {
    m_gyro.Reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.SetInverted(true);
    m_rightFollower.SetInverted(true);

    m_rightFollower.Follow(m_rightLeader);
    m_leftFollower.Follow(m_leftLeader);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // m_leftEncoder.SetControlFramePeriod(2 * std::numbers::pi * kWheelRadius /
    //                                   kEncoderResolution);
    // m_rightEncoder.SetControlFramePeriod(2 * std::numbers::pi * kWheelRadius /
    //                                    kEncoderResolution);

    m_leftEncoder.SetSelectedSensorPosition(0);
    m_rightEncoder.SetSelectedSensorPosition(0);
  }

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      std::numbers::pi};  // 1/2 rotation per second

  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t xSpeed,
             units::radians_per_second_t rot);
  void UpdateOdometry();
  void ResetOdometry(const frc::Pose2d& pose);
  frc::Pose2d GetPose() const;

 private:
  static constexpr units::length::meter_t kTrackWidth = 0.381_m;
  static constexpr double kWheelRadius = 0.0508;  // meters
  static constexpr int kEncoderResolution = 4096;

  WPI_TalonFX m_leftLeader{LEFT_FRONT_PORT};
  WPI_TalonFX m_leftFollower{LEFT_BACK_PORT};
  WPI_TalonFX m_rightLeader{RIGHT_FRONT_PORT};
  WPI_TalonFX m_rightFollower{RIGHT_BACK_PORT};

  TalonFX m_rightEncoder{5};
  TalonFX m_leftEncoder{3};

  frc2::PIDController m_leftPIDController{1.0, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{1.0, 0.0, 0.0};

  AHRS m_gyro{SerialPort::kUSB};

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{
      m_gyro.GetRotation2d(), units::length::meter_t{m_leftEncoder.GetSelectedSensorPosition()},
      units::length::meter_t{m_rightEncoder.GetSelectedSensorPosition()}};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SimpleMotorFeedforward<units::length::meters> m_feedforward{1_V, 3_V / 1_mps};

  
  // private double nativeUnitsToDistanceMeters(double sensorCounts){
  //   double motorRotations = (double)sensorCounts / kCountsPerRev;
  //   double wheelRotations = motorRotations / kGearRatio;
  //   double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
  //   return positionMeters;
  // }
};
