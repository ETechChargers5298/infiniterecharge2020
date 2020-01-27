/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Test Comment -Mr. Bianchi

package frc.robot.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autonomous;
import frc.robot.commands.GearShift;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.robot.Constants.JoystickConstants;
 
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();

  // Holds Autonomous Code
  private final Command m_autoCommand = new Autonomous();

  // Holds the Driver Controller Object
  XboxController driveController = new XboxController(JoystickConstants.DRIVECONTROLLER);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the Button Bindings
    configureButtonBindings();

    // Uses Left Joystick to Drive Robot
    driveTrain.setDefaultCommand(new ArcadeDrive(
      () -> (-1.0 * driveController.getY(GenericHID.Hand.kLeft)), 
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
    // Uses Button Y to Toggle the Drive Mode by Shifting Gears
    new JoystickButton(driveController, Button.kY.value).whenPressed(new GearShift(driveTrain));

    // Uses Left Bumper to Turn to -90 Degrees
    new JoystickButton(driveController, Button.kBumperLeft.value).whenPressed(new TurnToAngle(-90, driveTrain));

    // Uses Right Bumper to Turn to 90 Degrees
    new JoystickButton(driveController, Button.kBumperRight.value).whenPressed(new TurnToAngle(90, driveTrain));
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
