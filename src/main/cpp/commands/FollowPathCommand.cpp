
#include <commands/FollowPathCommand.h>

FollowPathCommand::FollowPathCommand(wpi::SmallString<64> path)
    : m_path(path)
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
