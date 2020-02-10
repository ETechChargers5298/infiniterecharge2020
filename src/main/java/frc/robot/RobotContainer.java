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
import frc.robot.utils.LightStrip;
import frc.robot.commands.Autonomous;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveGearShift;
import frc.robot.commands.DriveTurnToAngle;

import frc.robot.commands.IntakeChompDown;
import frc.robot.commands.IntakeRetractUp;
import frc.robot.commands.IntakeEat;
import frc.robot.commands.Shoot;
import frc.robot.commands.LiftReach;
import frc.robot.commands.LiftClimb;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Leveler;
import frc.robot.utils.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
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

  public final static LightStrip led = new LightStrip();   //Holds Addressable LED code
  public final static LimeLight limeLight = new LimeLight();

  // Holds Autonomous Code
  private final Command m_autoCommand = new Autonomous();

  // Holds the Driver Controller Object
  public final static XboxController driveController = new XboxController(JoystickConstants.DRIVECONTROLLER);
  public final static XboxController operatorController = new XboxController(JoystickConstants.OPERATORCONTROLLER);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the Button Bindings
    configureButtonBindings();

    // Uses Left Joystick to Drive Robot
    driveTrain.setDefaultCommand(new DriveArcade(
      () -> (-1.0 * driveController.getY(GenericHID.Hand.kLeft)), 
      () -> driveController.getX(GenericHID.Hand.kLeft)));

    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    // GEAR SHIFTING = Y-button to toggle between low gear & high gear
    new JoystickButton(driveController, Button.kY.value).whenPressed(new DriveGearShift());

    // SNAP TURNS = Left bumper for -90 turn, Right bumper for 90 turn
    new JoystickButton(driveController, Button.kBumperLeft.value).whenPressed(new DriveTurnToAngle(-90));
    new JoystickButton(driveController, Button.kBumperRight.value).whenPressed(new DriveTurnToAngle(90));
    //add forward & backward turns
    //add special turns to align with climb bar

    //INTAKE PISTONS = A-button chomp down, B-button retract
    new JoystickButton(operatorController, Button.kA.value).whenPressed(new IntakeChompDown());
    new JoystickButton(operatorController, Button.kB.value).whenPressed(new IntakeRetractUp());
  
    //INTAKE WHEELS = hold
    new JoystickButton(operatorController, Button.kX.value).whileHeld(new IntakeEat(1.0));
    //new JoystickButton(operatorController, Button.kX.value).whenReleased(new IntakeSetWheelSpeed(0));

    //SHOOTER = X-button to spin shooting wheel
    new JoystickButton(driveController, Button.kX.value).whenHeld(new Shoot());
    //add command to manually load PowerCells into the shooter
    //add combo-command to automatically start up shooter wheels and launch 5 PowerCells
    //add command to adjust the angle of the shooter

    //LIFT = A-button to reach, B-button to climb
    new JoystickButton(driveController, Button.kA.value).whenPressed(new LiftReach());
    new JoystickButton(driveController, Button.kB.value).whenPressed(new LiftClimb());

    //LEVELER
    //add command to manually slide right
    //add command to manually slide left
    //add command to automatically level based on navx

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
