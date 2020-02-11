/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Holds CAN IDs from SparkMax speed controllers
        public static final int MOTOR_LEFT_ZERO = 1;
        public static final int MOTOR_LEFT_ONE = 4;
        public static final int MOTOR_RIGHT_ZERO = 2;
        public static final int MOTOR_RIGHT_ONE = 6;

        // Holds wheel inversion boolean (One should be true and the other one should be false)
        public static final boolean LEFT_INVERSION = true;
        public static final boolean RIGHT_INVERSION = false;

        // Wheel Data Info
        public static final double WHEEL_RADIUS = 6; // Inches
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_RADIUS * 2.0;

        // Encoder Resolution
        public static final int DRIVE_ENCODER_RESOLUTION = 8192;

        // DriveTrain Wheel Data
        public static final double DRIVE_TRACK_WIDTH = 12; // Inches
        public static final double DRIVE_WHEEL_RADIUS = 6; // Inches

        // Max Speed
        public static final double MAX_SPEED = 1;

        // Max Velocities
        public static final double MAX_VELOCITY = 3.0; // Meters per Second
        public static final double MAX_TURN_VELOCITY = 2 * Math.PI; // Radians Per Second

        // PID Constants for Left Wheels on High Speed
        public static final double LEFT_SPEED_DRIVE_P = 1;
        public static final double LEFT_SPEED_DRIVE_I = 0;
        public static final double LEFT_SPEED_DRIVE_D = 0;

        // PID Constants for Right Wheels on High Speed
        public static final double RIGHT_SPEED_DRIVE_P = 1;
        public static final double RIGHT_SPEED_DRIVE_I = 0;
        public static final double RIGHT_SPEED_DRIVE_D = 0;

        // PID Constants for Left Wheels on High Torque
        public static final double LEFT_TORQUE_DRIVE_P = 1;
        public static final double LEFT_TORQUE_DRIVE_I = 0;
        public static final double LEFT_TORQUE_DRIVE_D = 0;

        // Feed Forward Constants
        public static final double DRIVE_SPEED_STATIC_GAIN = 1;
        public static final double DRIVE_SPEED_VELOCITY_GAIN = 3;
        public static final double DRIVE_TORQUE_STATIC_GAIN = 1;
        public static final double DRIVE_TORQUE_VELOCITY_GAIN = 3;

        // Ball Shifters on Gearbox.
        public static final int SHIFTER_PORT_ONE = 0;
        public static final int SHIFTER_PORT_TWO = 1;
    }

    public static final class JoystickConstants {
        // Deadbands ignore smaller inputs from joystick for smoother driving
        public static final double DEADBAND = 0.01;

        // Sets JoyStick Sensitivity (Even Numbers ONLY)
        public static double JOYSTICK_SENSITIVITY = 8;

        // Numbers for the 2 Xbox controllers
        public static final int DRIVECONTROLLER = 0;
        public static final int OPERATORCONTROLLER = 1;

        public static final double SPEED_LIMIT = 3; // (1 / Speed_Limit) Seconds from 0 to 1
        public static final double ROT_LIMIT = 3; // (1 / Rot_Limit) Seconds from 0 to 1
    }

    public static final class TurnToAngleConstants {
        // PID Constants for Turning
        public static final double TURN_P = 1;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0;

        // Turning Tolerance
        public static final double TURN_TOLERANCE = 5; // Degrees
        public static final double TURN_RATE_TOLERANCE = 10; // Degrees per Second
    }
    
    public static final class LevelConstants {
        // CAN ID for Motors
        public static final int LEVEL_MOTOR_ID = 9;

        // Max Velocity of Motor
        public static final double MAX_VELOCITY = 0.25;

        // Max Acceleration of Motor
        public static final double MAX_ACCELERATION = 0.15;

        // PID Constants for Leveler
        public static final double LEVEL_P = 1;
        public static final double LEVEL_I = 0;
        public static final double LEVEL_D = 0;
        
        // Goal of Leveler in Degrees
        public static final double GOAL = 0.0;

        // Sets Tolerance for PID Setpoint
		public static final double DEGREE_TOLERANCE = 5; // Degrees
		public static final double VELOCITY_TOLERANCE = 1; // Rotation per Second
    }

    public static final class LimeLightConstants {
        // PipeLine ID of Shooter
        public static final int SHOOTER_PIPELINE = 0;

        // Holds Proportional Constant for Aiming
        public static final double AIM_P = 1;

        // Holds the Minium Power to Move Robot
        public static final double AIM_MIN_SPEED = 0.1;
    }

    public static final class IntakeConstants {
        // Holds The ID of the Motor
        public static final int INTAKE_MOTOR_ID = 31;

        // Inversion of the Motor
        public static final boolean INTAKE_MOTOR_INVERSION = false;

        // Max Speed of the Intake Motors
        public static final double INTAKE_MAX_SPEED = 1;

        // Holds The Solenoid Ports
        public static final int INTAKE_PORT_ZERO = 3;
        public static final int INTAKE_PORT_ONE = 4;
    }

    public static final class LiftConstants {
        // Holds the Solenoid Ports
        public static final int LIFT_PORT_ZERO = 5;
        public static final int LIFT_PORT_ONE = 6;
    }

    public static final class ShooterConstants {
        // Holds the ID of the Motor
        public static final int SHOOTER_MOTOR_ID = 32;

        // Inversion of the Motor
        public static final boolean SHOOTER_MOTOR_INVERSION = false;

        // Max Speed of the Motors
        public static final double SHOOTER_MAX_SPEED = 1;
        // Holds the Solenoid Ports
        public static final int SHOOTER_PORT_ZERO = 7;
        public static final int SHOOTER_PORT_ONE = 8;
    }

    public static final class LightStripConstants {
        public static final int PWM_PORT = 9;
        public static final int NUM_PIXELS = 60;

    }

}
