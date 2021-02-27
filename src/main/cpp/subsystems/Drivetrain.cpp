#include "subsystems/Drivetrain.h"
#include <Robot.h>

#include <ctime>
#include <unistd.h>


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
    m_leftEncoder.SetVelocityConversionFactor(kConversionFactor / 60.0);
    m_rightEncoder.SetVelocityConversionFactor(kConversionFactor / 60.0);

    std::string filename = "/home/lvuser/odometry.csv";
    char buf[100];
    std::cout << std::string(getcwd(buf, 100)) << filename << std::endl;
    m_logFile.open(filename);
    if(!m_logFile.good() || !m_logFile.is_open())
    {
        std::cout << "Log File failed to open" << std::endl;
    }
    m_logFile << "angle,left,right,rotation,positionX,positionY,leftVelocity,rightVelocity\n" << std::flush;

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

void Drivetrain::SetVolts(units::volt_t left, units::volt_t right)
{
    m_leftGroup.SetVoltage(left);
    m_rightGroup.SetVoltage(-right);
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
    auto right = units::meter_t(-m_rightEncoder.GetPosition());

    auto pos = m_odometry.Update(rot, left, right);

    if((m_steps % kLogInterval) == 0)
    {
        auto leftV = units::meters_per_second_t(m_leftEncoder.GetVelocity());
        auto rightV = units::meters_per_second_t(-m_rightEncoder.GetVelocity());

        m_logFile 
            << std::to_string(m_navX.GetAngle()) << ","
            << std::to_string(double(left)) << ","
            << std::to_string(double(right)) << ","
            << std::to_string(double(rot.Degrees())) << ","
            << std::to_string(double(pos.X())) << ","
            << std::to_string(double(pos.Y())) << ","
            << std::to_string(double(leftV)) << ","
            << std::to_string(double(rightV)) << ","
            << "\n";
        m_logFile.flush();
    }
    m_steps++;
}

double Drivetrain::GetAngle() // units::degree_t
{
    return m_navX.GetAngle();
}

frc::Rotation2d Drivetrain::GetRotation()
{
    return m_navX.GetRotation2d();
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds()
{
    return{
        units::meters_per_second_t(m_leftEncoder.GetVelocity()),
        units::meters_per_second_t(-m_rightEncoder.GetVelocity())
    };
}

frc::Pose2d Drivetrain::GetPose()
{
    return m_odometry.GetPose();
}

void Drivetrain::ResetOdometry(frc::Pose2d pose)
{
    m_leftEncoder.SetPosition(0);
    m_rightEncoder.SetPosition(0);
    m_odometry.ResetPosition(pose, GetRotation());
}