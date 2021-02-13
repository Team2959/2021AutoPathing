#include "subsystems/Drivetrain.h"
#include <Robot.h>

#include <ctime>

bool FileExists(std::string filename)
{
    std::ifstream file;
    file.open(filename);
    return file.good();
}

Drivetrain::Drivetrain()
{
    m_leftFollower1.Follow(m_leftPrimary);
    m_leftFollower2.Follow(m_leftPrimary);
    m_rightFollower1.Follow(m_rightPrimary);
    m_rightFollower2.Follow(m_rightPrimary);

    m_leftPrimary.SetInverted(false);
    m_rightPrimary.SetInverted(false);
    m_differentialDrive.SetRightSideInverted(true);

    m_leftEncoder.SetPositionConversionFactor(kConversionFactor);
    m_rightEncoder.SetPositionConversionFactor(kConversionFactor);

    std::string filename;
    for(int i = 0; ; i++)
    {
        filename = std::string("odometry") + std::to_string(i) + std::string(".csv");
        if(!FileExists(filename)) break;
    }
    m_logFile.open(filename);
    m_logFile << "rotation,left,right,positionX,positionY\n";

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
    auto rot = GetRotation();
    auto left = units::meter_t(m_leftEncoder.GetPosition());
    auto right = units::meter_t(m_rightEncoder.GetPosition());
    auto pos = m_odometry.Update(rot, left, right);
    if((m_steps % kLogInterval) == 0)
    {
        m_logFile 
            << std::to_string(double(rot.Degrees())) << ","
            << std::to_string(double(left)) << ","
            << std::to_string(double(right)) << ","
            << std::to_string(double(pos.X())) << ","
            << std::to_string(double(pos.Y())) << "\n";
    }
    m_steps++;
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

void Drivetrain::ResetOdometry(frc::Pose2d pose, frc::Rotation2d rotation)
{
    m_leftEncoder.SetPosition(0);
    m_rightEncoder.SetPosition(0);
    m_odometry.ResetPosition(pose, rotation);
}