/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Test Comment -Mr. Bianchi

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.LightStrip;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveGearShift; //could be deleted
import frc.robot.commands.DriveHighTorque;
import frc.robot.commands.DriveHighSpeed;
import frc.robot.commands.Shoot;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.IntakeDrop;
import frc.robot.commands.IntakeGrabBall;
import frc.robot.commands.IntakeReleaseBall;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.Level;
import frc.robot.commands.LiftClimb;
import frc.robot.commands.LiftReach;
import frc.robot.commands.LoaderLoad;
import frc.robot.commands.LoaderRelease;
import frc.robot.commands.MoveLevel;
import frc.robot.autoCommands.AutoDriveStraight;
import frc.robot.autoCommands.AutoTripleShot;
import frc.robot.autoCommands.AutoDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leveler;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.JoystickConstants;
 
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static DriveTrain driveTrain = new DriveTrain();
  public final static Intake intake = new Intake();
  public final static Shooter shooter = new Shooter();
  public final static Lift lift = new Lift();
  public final static Leveler leveler = new Leveler();
  public final static Loader loader = new Loader();

  public final static LightStrip led = new LightStrip();
  public final static LimeLight limeLight = new LimeLight();

  // Holds the Driver Controller Object
  public final static XboxController driveController = new XboxController(JoystickConstants.DRIVECONTROLLER);
  public final static XboxController operatorController = new XboxController(JoystickConstants.OPERATORCONTROLLER);

  //Sendable Chooser
  public SendableChooser<CommandGroupBase> autoChooser;
  public static Command chosenAutoCommand;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the Button Bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(new DriveArcade(
      () -> (-1.0 * driveController.getY(GenericHID.Hand.kLeft)), 
      () -> driveController.getX(GenericHID.Hand.kLeft)));
    
    leveler.setDefaultCommand(new MoveLevel(
      () -> operatorController.getX(GenericHID.Hand.kRight)
    ));

      autoChooser = new SendableChooser<CommandGroupBase>();
      autoChooser.addOption("Only Drive Straight", new AutoDrive());
      autoChooser.addOption("Only Shoot", new AutoTripleShot());

    // Reset Sensors
    driveTrain.reset();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    new JoystickButton(operatorController, Button.kX.value).whenPressed(new LoaderLoad());

    new JoystickButton(operatorController, Button.kY.value).whenPressed(new LoaderRelease());
    
    // GEAR SHIFTING = Right Bumper/Left Bumper
    new JoystickButton(driveController, Button.kBumperLeft.value).whenPressed(new DriveHighTorque());

    new JoystickButton(driveController, Button.kBumperRight.value).whenPressed(new DriveHighSpeed());
    // LIFT CLIMB = Left Bumper
    //new JoystickButton(operatorController, Button.kA.value).whenPressed(new LiftClimb(lift));

    new JoystickButton(driveController, Button.kA.value).whenHeld(new Shoot());
    // LIFT Reach = Left Bumper

    // LEVEL = Right stick x-axis
    //new JoystickButton(operatorController, Button.k).whenPressed(new Level());

    // GRAB BALL = B-button
    new JoystickButton(operatorController, Button.kB.value).whileHeld(new IntakeGrabBall(intake), true);
    
    // INTAKE RETRACT = Right Trigger
    //new JoystickButton(operatorController, Button.k).whenPressed(new IntakeRetract(intake));

    // INTAKE DROP = RIGHT BUMPER
    new JoystickButton(operatorController, Button.kBumperRight.value).whenPressed(new IntakeDrop(intake));

    // SHOOT = X-Button
    new JoystickButton(driveController, Button.kX.value).whenPressed(new Shoot());

    //Spit Ball
    new JoystickButton(operatorController, Button.kA.value).whileHeld(new IntakeReleaseBall(intake), true);

    new JoystickButton(operatorController, Button.kBumperLeft.value).whenPressed(new LiftReach(lift));
    
    //LOAD = Y-button
    //new JoystickButton(operatorController, Button.kY.value).whenPressed(new )
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Whichever command is assigned to chosenAutoCommand will run in autonomous
    return chosenAutoCommand;
  }
}
