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
    public static final class SparkConstants {
        // Holds CAN IDs from SparkMax Speed Controllers
        public static final int MOTOR_LEFT_A = 1;
        public static final int MOTOR_LEFT_B = 2;
        public static final int MOTOR_LEFT_C = 5; // Not Used
        
        public static final int MOTOR_RIGHT_A = 3;
        public static final int MOTOR_RIGHT_B = 4;
        public static final int MOTOR_RIGHT_C = 6; // Not Used
        
        // For Mechanisms Other than DriveTrain
        public static final int MOTOR_INTAKE = 7;
        public static final int MOTOR_LOADER = 8;
        public static final int MOTOR_ANGLER = 9;
        public static final int MOTOR_SHOOTER = 10;
        public static final int MOTOR_LEVELER = 11;
    }

    public static final class SolenoidConstants {
        // Ball Shifters on Gearbox    
        public static final int SHIFTER_PORT_A = 0;
        public static final int SHIFTER_PORT_B = 1;

        // Intake Pistons
        public static final int INTAKE_PORT_A = 3;
        public static final int INTAKE_PORT_B = 2;

        // Lift Pistons
        public static final int LIFT_PORT_A = 5;
        public static final int LIFT_PORT_B = 4;
    }

    public static final class DriveConstants {

        // Encoder Raw to Inch Multiplier
        public static final double RAW_PER_INCH = 1075;

        //Angles for the robot to face based on the NavX
        public static final double ANGLE_FORWARD = 0.0;
        public static final double ANGLE_RIGHT = 90.0;
        public static final double ANGLE_BACKWARDS = 179.9;
        public static final double ANGLE_LEFT = -90.0;
        public static final double ANGLE_MIDDLE_APPROACH = -22.5;
        public static final double ANGLE_SIDE_APPROACH = 22.5;

        // Holds Wheel Inversion
        public static final boolean LEFT_INVERSION = true;
        public static final boolean RIGHT_INVERSION = true;

        // Encoder Resolution
        public static final int DRIVE_ENCODER_RESOLUTION = 8192;

        // DriveTrain Wheel Data
        public static final double DRIVE_TRACK_WIDTH = 12; // Inches
        public static final double DRIVE_WHEEL_RADIUS = 6; // Inches
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * DRIVE_WHEEL_RADIUS * 2.0;

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

        // PID Constants for Right Wheels on High Torque
        public static final double RIGHT_TORQUE_DRIVE_P = 1;
        public static final double RIGHT_TORQUE_DRIVE_I = 0;
        public static final double RIGHT_TORQUE_DRIVE_D = 0;

        // Feed Forward Constants
        public static final double DRIVE_SPEED_STATIC_GAIN = 1;
        public static final double DRIVE_SPEED_VELOCITY_GAIN = 3;
        public static final double DRIVE_TORQUE_STATIC_GAIN = 1;
        public static final double DRIVE_TORQUE_VELOCITY_GAIN = 3;

        // Inverts Encoders
		public static final boolean LEFT_ENCODER_INVERSION = true;
		public static final boolean RIGHT_ENCODER_INVERSION = false;
    }

  

    public static final class JoystickConstants {
        // Deadbands ignore smaller inputs from joystick for smoother driving
        public static final double DEADBAND = 0.01;

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
        // Inversion of the Motor
        public static final boolean INTAKE_MOTOR_INVERSION = false;

        // Max Speed of the Intake Motors
        public static final double INTAKE_MAX_SPEED = 1;
    }

    public static final class ShooterConstants {
        // Inversion of the Motor
        public static final boolean SHOOTER_MOTOR_INVERSION = false;

        // Max Speed of the Motors
        public static final double SHOOTER_MAX_SPEED = 1.0;
        public static final double LOAD_SPEED = -1.0;

        // Inversion of the Angler Motor
        public static final boolean ANGLER_MOTOR_INVERSION = true;

        // Max Speed of the Angler Motor
        public static final double ANGLER_MAX_SPEED = 1.0;

        //Angles for the Shooter
        public static final int TRENCH_ANGLE = 1200; // 45
        public static final int SIDE_START_ANGLE = 1000; // 55
        public static final int SIDE_MIDDLE_ANGLE = 750; // 52
        public static final int WALL_ANGLE = 100; // 68
        
        public static final int EXTREME_LOW = 2000;
        public static final int EXTREME_HIGH = 0;
    }

    public static final class LightStripConstants {
        // PWM Port for REV BLINKIN LEDs
        public static final int PWM_PORT = 9;

        // Number of LEDs on Strip
        public static final int NUM_PIXELS = 60;
    }
}
