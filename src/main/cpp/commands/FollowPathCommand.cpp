
#include <commands/FollowPathCommand.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>

FollowPathCommand::FollowPathCommand(Drivetrain* drivetrain, wpi::SmallString<64> path)
    : m_drivetrain(drivetrain), m_path(path)
{
}

void FollowPathCommand::Initialize()
{
    wpi::SmallString<64> deployDirectory;
    frc::filesystem::GetDeployDirectory(deployDirectory);
    wpi::sys::path::append(deployDirectory, "paths");
    wpi::sys::path::append(deployDirectory, m_path);

    frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    

}
