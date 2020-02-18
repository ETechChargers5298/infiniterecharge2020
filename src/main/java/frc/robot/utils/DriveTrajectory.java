package frc.robot.utils;

import java.util.Arrays;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class DriveTrajectory {

    private TrajectoryConfig config;

    public DriveTrajectory() {
        config = new TrajectoryConfig(2, 2); // Max Velocity, Max Acceleration m/s
    }

    public Trajectory driveForward() {
        return TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())), config);
    }
}