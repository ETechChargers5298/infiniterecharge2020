package frc.robot.utils;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveTrajectory {

    private TrajectoryConfig config;

    public DriveTrajectory() {
        config = new TrajectoryConfig(2, 2); // Max Velocity, Max Acceleration m/s
    }

    public RamseteCommand autoForward(DriveTrain driveTrain) {
        return trajectoryTorqueCommand(
            driveTrain, 
            TrajectoryGenerator.generateTrajectory(
                Arrays.asList(
                   new Pose2d(),
                   new Pose2d(1.0, 0, new Rotation2d())
                ), 
                config
            )
        );
    }
    
    // Returns a Autonomous Command Using the Trajectory Given for High Speed Mode
    public RamseteCommand trajectorySpeedCommand(DriveTrain driveTrain, Trajectory trajectory) {
        return new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(2.0, 0.7),
            driveTrain.getSpeedFeedforward(),
            driveTrain.getKinematics(),
            driveTrain::getSpeeds,
            driveTrain.getLeftSpeedPID(),
            driveTrain.getRightSpeedPID(),
            driveTrain::voltDrive, 
            driveTrain
        );
    }

    // Returns a Autonomous Command Using the Trajectory Given for High Torque Mode
    public RamseteCommand trajectoryTorqueCommand(DriveTrain driveTrain, Trajectory trajectory) {
        return new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(2.0, 0.7),
            driveTrain.getTorqueFeedforward(),
            driveTrain.getKinematics(),
            driveTrain::getSpeeds,
            driveTrain.getLeftTorquePID(),
            driveTrain.getRightTorquePID(),
            driveTrain::voltDrive, 
            driveTrain
        );
    }
}