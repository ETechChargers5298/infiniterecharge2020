/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;

public class AutoDriveStraight extends PIDCommand {
  /**
   * Creates a new AutonomousDriveStraight.
   */
  
  private DriveTrain drive;
  private double speed;
  private double inchDistance;
  private double startDistance;
  private double rawDistance;
  private boolean finished;

  public AutoDriveStraight(DriveTrain drive, double speed, double distanceInches) {
    super(
        // The controller that the command will use
        new PIDController(0.005, 0, 0),
        // This should return the measurement
        RobotContainer.driveTrain::getInches,
        // This should return the setpoint (can also be a constant)
        distanceInches,
        // This uses the output
        output -> RobotContainer.driveTrain.drive(speed, 0.0),
          // Use the output here
          RobotContainer.driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(100);

    this.drive = drive;
    this.speed = speed;
    this.inchDistance = distanceInches;
    //this.rawDistance = distanceInches / 25 * 1000;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //startDistance = Timer.getFPGATimestamp();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drive.arcadeDrive(speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  //@Override
  //public void end(boolean interrupted) {




  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  
  }
}
