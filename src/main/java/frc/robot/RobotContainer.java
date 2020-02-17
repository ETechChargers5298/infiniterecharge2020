/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Test Comment -Mr. Bianchi

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.TriggerButton;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Constants.JoystickConstants;

import frc.robot.utils.LightStrip;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveGearShift; //could be deleted
import frc.robot.commands.DriveHighTorque;
import frc.robot.commands.DriveReset;
import frc.robot.commands.DriveHighSpeed;
import frc.robot.commands.ShooterShoot;
import frc.robot.commands.ShooterLoadOnly;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.IntakeChomp;
import frc.robot.commands.IntakeEat;
import frc.robot.commands.IntakeSpit;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.Level;
import frc.robot.commands.LiftClimb;
import frc.robot.commands.LiftReach;
import frc.robot.commands.MoveLevel;
import frc.robot.commands.ShooterAngle;
import frc.robot.commands.AutoDriveStraight;
import frc.robot.commands.AutoShooterAngle;
import frc.robot.commandGroups.AutoDriveOnly;
import frc.robot.commandGroups.AutoTripleShot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leveler;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.LimeLight;

 
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

    configureAxes();

    autoChooser = new SendableChooser<CommandGroupBase>();
    autoChooser.addOption("Only Drive Straight", new AutoDriveOnly());
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

    // DRIVE HIGH SPEED & HIGH TORQUE = Right Bumper/ Right Trigger
    new JoystickButton(driveController, Button.kBumperRight.value).whenPressed(new DriveHighSpeed(driveTrain));
    new TriggerButton(driveController, Hand.kRight).whileActiveOnce(new DriveHighTorque(driveTrain));

    // DRIVE TURN-TO-ANGLE = POVs
    new POVButton(driveController, 0).whileHeld(new DriveTurnToAngle(Constants.DriveConstants.ANGLE_FORWARD));
    new POVButton(driveController, 90).whileHeld(new DriveTurnToAngle(Constants.DriveConstants.ANGLE_RIGHT));
    new POVButton(driveController, 180).whileHeld(new DriveTurnToAngle(Constants.DriveConstants.ANGLE_BACKWARDS));
    new POVButton(driveController, 270).whileHeld(new DriveTurnToAngle(Constants.DriveConstants.ANGLE_LEFT));
    new JoystickButton(driveController, Button.kY.value).whenPressed(new DriveReset(driveTrain));

    // INTAKE EAT & SPIT = B/A buttons
    new JoystickButton(operatorController, Button.kB.value).whileHeld(new IntakeEat(intake), true);
    new JoystickButton(operatorController, Button.kA.value).whileHeld(new IntakeSpit(intake), true);

    // INTAKE CHOMP & RETRACT = X/Y buttons
    new JoystickButton(operatorController, Button.kX.value).whenPressed(new IntakeChomp(intake));
    new JoystickButton(operatorController, Button.kY.value).whenPressed(new IntakeRetract(intake));

    // SHOOTER ANGLER = POVs
    // new POVButton(operatorController, 0).whenPressed(new AutoShooterAngle(shooter, Constants.ShooterConstants.TRENCH_ANGLE));
    // new POVButton(operatorController, 180).whenPressed(new AutoShooterAngle(shooter, Constants.ShooterConstants.WALL_ANGLE));

    //SHOOTER LOAD = RIGHT TRIGGER
    new TriggerButton(operatorController, Hand.kRight).whileActiveOnce(new DriveHighTorque(driveTrain));

    //SHOOTER SPIN = RIGHT BUMPER
    new JoystickButton(operatorController, Button.kBumperRight.value).whileHeld(new ShooterShoot(shooter));

    // LIFT REACH = LB button
    new JoystickButton(operatorController, Button.kBumperLeft.value).whenPressed(new LiftReach(lift));

    // LIFT CLIMB = Left Trigger
    new TriggerButton(operatorController, Hand.kLeft).whileActiveOnce(new LiftClimb(lift));


  }


  public void configureAxes(){

    //DRIVE WITH JOYSTICKS
    driveTrain.setDefaultCommand(new DriveArcade(
      () -> (-1.0 * driveController.getY(Hand.kLeft)), 
      () -> driveController.getX(Hand.kLeft)));
    
    //SHOOTER ANGLER MANUAL = Left Y-Axis
    shooter.setDefaultCommand(new ShooterAngle(
      shooter,
      () -> operatorController.getY(Hand.kLeft)
    ));

    // LEVEL = Right stick x-axis
    leveler.setDefaultCommand(new MoveLevel(
      leveler,
      () -> operatorController.getX(Hand.kRight)
    ));

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
