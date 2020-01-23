/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Test Comment -Mr. Bianchi

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GearShift;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.BallShifter;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final BallShifter gearShift = new BallShifter();
  private final NavX navX = new NavX();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // Creates a Driver Controller Varibale
  XboxController driveController = new XboxController(Constants.DRIVECONTROLLER);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(new DriveCommand(
      () -> driveController.getY(GenericHID.Hand.kLeft), 
      () -> driveController.getX(GenericHID.Hand.kLeft),
      driveTrain));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    // Button Y when pressed changes Gear Shifts
    new JoystickButton(driveController, Button.kY.value).whenPressed(new GearShift(gearShift));

    new JoystickButton(driveController, Button.kBumperLeft.value)
    .whenPressed(new TurnToAngle(-90.0, driveTrain, navX));

    new JoystickButton(driveController, Button.kBumperRight.value)
    .whenPressed(new TurnToAngle(-90.0, driveTrain, navX));
}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
