#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#include <AHRS.h>
#include "RobotMap.h"
#include "utility/AngleConversion.h"

#include <rev/CANSparkMax.h>

class Drivetrain : public frc2::SubsystemBase
{
private:
    const double kOpenLoopRampRate = 0.25;
    const double kCurrentLimit = 50;
    const double kDefaultAutoKp = 0.001;
    const double kDefaultLimitAngle = 0.5;
    const double kDefaultMinSpeed = 0.045;

    rev::CANSparkMax m_leftPrimary{kDrivetrainLeftPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollower1{kDrivetrainLeftFollower1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollower2{kDrivetrainLeftFollower2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANPIDController m_leftPID{m_leftPrimary};
    rev::CANEncoder m_leftEncoder{m_leftPrimary};

    rev::CANSparkMax m_rightPrimary{kDrivetrainRightPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollower1{kDrivetrainRightFollower1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollower2{kDrivetrainRightFollower2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANPIDController m_rightPID{m_rightPrimary};
    rev::CANEncoder m_rightEncoder{m_rightPrimary};

    frc::SpeedControllerGroup m_leftGroup {m_leftPrimary};
    frc::SpeedControllerGroup m_rightGroup {m_rightPrimary};

    frc::DifferentialDrive m_differentialDrive {m_leftGroup, m_rightGroup};

    AHRS m_navX{frc::SPI::kMXP};
    frc::DifferentialDriveOdometry m_odometry{frc::Rotation2d{static_cast<units::radian_t>(RadiansToDegrees(m_navX.GetAngle()))}};

    void SetupSparkMax(rev::CANSparkMax* controller);

public:
    Drivetrain();

    void CurvatureDrive(double speed, double rotation, bool quickTurn);

    
    /**
    * Will be called periodically whenever the CommandScheduler runs.
    */
    void Periodic() override;

    double GetAngle();
    double GetPosition();
    void SetVolts(units::volt_t left, units::volt_t right); 
};