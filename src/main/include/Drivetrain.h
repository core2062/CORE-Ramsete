// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/GroupMotorControllers.h>

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
    // m_rightLeader.SetInverted(true);
    // m_rightFollower.Follow(m_rightLeader);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_leftEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
    //                                   kEncoderResolution);
    // m_rightEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
    //                                    kEncoderResolution);
  

    m_leftLeader.SetSelectedSensorPosition(0);
    m_rightLeader.SetSelectedSensorPosition(0);

    m_leftLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 10, 0);
    m_rightLeader.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 10, 0);

    m_leftLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);
    m_rightLeader.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 0);

    m_rightFollower.Follow(m_rightLeader);
    m_leftFollower.Follow(m_leftLeader);

    m_rightLeader.SetInverted(TalonFXInvertType::Clockwise);
    m_rightFollower.SetInverted(TalonFXInvertType::FollowMaster);

    m_leftLeader.SetInverted(TalonFXInvertType::CounterClockwise);
    m_leftFollower.SetInverted(TalonFXInvertType::FollowMaster);
  }

  double NativeUnitsToDistanceMeters(double sensorCounts);

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
  static constexpr units::meter_t kTrackWidth = 1.1684_m;
  static constexpr double kWheelRadius = 0.1524;  // meters
  static constexpr double kGearRatio = 12.7;
  static constexpr int kEncoderResolution = 2048;

  WPI_TalonFX m_rightLeader{1};
  WPI_TalonFX m_rightFollower{2};
  WPI_TalonFX m_leftLeader{3};
  WPI_TalonFX m_leftFollower{4};


  // TalonFX m_rightEncoder{1};
  // TalonFX m_leftEncoder{3};

  // frc::PWMSparkMax m_leftLeader{1};
  // frc::PWMSparkMax m_leftFollower{2};
  // frc::PWMSparkMax m_rightLeader{3};
  // frc::PWMSparkMax m_rightFollower{4};

  // frc::MotorControllerGroup m_leftGroup{m_leftLeader, m_leftFollower};
  // frc::MotorControllerGroup m_rightGroup{m_rightLeader, m_rightFollower};

  // frc::Encoder m_leftEncoder{0, 1};
  // frc::Encoder m_rightEncoder{2, 3};

  frc2::PIDController m_leftPIDController{1.0, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{1.0, 0.0, 0.0};

  AHRS m_gyro{frc::SerialPort::kUSB};

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{
      m_gyro.GetRotation2d(), units::meter_t{(((double)m_leftLeader.GetSelectedSensorPosition(0) / kEncoderResolution) / kGearRatio)*(2 * std::numbers::pi * kWheelRadius)},
      units::meter_t{(((double)m_rightLeader.GetSelectedSensorPosition(0) / kEncoderResolution) / kGearRatio)*(2 * std::numbers::pi * kWheelRadius)}};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{0.11578_V, 2.8026 * 1_V * 1_s / 1_m, 0.2154 * 1_V * 1_s * 1_s / 1_m};
};
