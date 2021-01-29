#include "subsystems/Drivetrain.h"
#include <Robot.h>

Drivetrain::Drivetrain()
{
    m_leftFollower1.Follow(m_leftPrimary);
    m_leftFollower2.Follow(m_leftPrimary);
    m_rightFollower1.Follow(m_rightPrimary);
    m_rightFollower2.Follow(m_rightPrimary);

    m_leftPrimary.SetInverted(false);
    m_rightPrimary.SetInverted(false);
    m_differentialDrive.SetRightSideInverted(true);

    SetupSparkMax(&m_leftPrimary);
    SetupSparkMax(&m_rightPrimary);
    SetupSparkMax(&m_leftFollower1);
    SetupSparkMax(&m_leftFollower2);
    SetupSparkMax(&m_rightFollower1);
    SetupSparkMax(&m_rightFollower2);
}

void Drivetrain::SetupSparkMax(rev::CANSparkMax* controller)
{
    controller->ClearFaults();
    controller->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    controller->SetSmartCurrentLimit(kCurrentLimit);
    controller->SetOpenLoopRampRate(kOpenLoopRampRate);
}

double Drivetrain::GetAngle() // units::degree_t
{
    return m_navX.GetAngle();
}

void Drivetrain::SetVolts(units::volt_t left, units::volt_t right)
{
    m_leftGroup.SetVoltage(left);
    m_rightGroup.SetVoltage(right);
    m_differentialDrive.Feed();
}

void Drivetrain::CurvatureDrive(double speed, double rotation, bool quickTurn)
{
    m_differentialDrive.CurvatureDrive(speed, rotation, quickTurn);
}

void Drivetrain::Periodic()
{
    m_odometry.Update(GetRotation(), units::meter_t(m_leftEncoder.GetPosition()), units::meter_t(m_rightEncoder.GetPosition()));
}

frc::Rotation2d Drivetrain::GetRotation()
{
    return frc::Rotation2d(units::angle::degree_t(RadiansToDegrees(m_navX.GetAngle())));
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds()
{
    return{
        units::meters_per_second_t(m_leftEncoder.GetVelocity()),
        units::meters_per_second_t(m_rightEncoder.GetVelocity())
    };
}

frc::Pose2d Drivetrain::GetPose()
{
    return m_odometry.GetPose();
}