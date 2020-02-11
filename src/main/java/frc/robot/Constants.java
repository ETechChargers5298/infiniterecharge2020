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

        // Encoder Resolution
        public static final int DRIVE_ENCODER_RESOLUTION = 8192;

        // DriveTrain Wheel Data
        public static final double DRIVE_TRACK_WIDTH = 12; // Inches
        public static final double DRIVE_WHEEL_RADIUS = 6; // Inches

        // Wheel Data Info
        public static final double WHEEL_RADIUS = 6; // Inches
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_RADIUS * 2.0;

        // Holds wheel inversion boolean (One should be true and the other one should be false)
        public static final boolean LEFT_INVERSION = true;
        public static final boolean RIGHT_INVERSION = true;

        // Constant that is multiplied to power to lower max possible power
        public static final double SPEED_MULTIPLIER = 1.0;

        // Ball Shifters on Gearbox.
        public static final int SHIFTER_PORT_ONE = 0;
        public static final int SHIFTER_PORT_TWO = 1;

        // Max Speed for Turn
        public static final double MAX_SPEED = 1;
        // Max Speed
        public static final double MAX_VELOCITY = 3.0; // Meters per Second
        public static final double MAX_TURN_SPEED = 2 * Math.PI; // Rotation Per Second

        // PID Constants for Left Wheels
        public static final double LEFT_DRIVE_P = 1;
        public static final double LEFT_DRIVE_I = 0;
        public static final double LEFT_DRIVE_D = 0;

        // PID Constants for Right Wheels
        public static final double RIGHT_DRIVE_P = 1;
        public static final double RIGHT_DRIVE_I = 0;
        public static final double RIGHT_DRIVE_D = 0;

        // Feed Forward Constants
        public static final double DRIVE_STATIC_GAIN = 1;
        public static final double DRIVE_VELOCITY_GAIN = 3;

        // Lift Solenoid Constants
        public static final int LIFTER_PORT_THREE = 2;
        public static final int LIFTER_PORT_FOUR = 3;

        // Intake Solenoid Constants 
        public static final int INTAKE_PORT_FIVE = 4;
        public static final int INTAKE_PORT_SIX = 5;

		public static final int MOTOR_INTAKE_WHEEL = 0;
    }

    public static final class MechanismConstants {

        // Holds CAN IDs from SparkMax speed controllers
        public static final int MOTOR_LEFT_A = 1;
        public static final int MOTOR_LEFT_B = 2;
        public static final int MOTOR_LEFT_C = 3;
        
        public static final int MOTOR_RIGHT_A = 4;
        public static final int MOTOR_RIGHT_B = 5;
        public static final int MOTOR_RIGHT_C = 6;
        
        public static final int MOTOR_INTAKE_WHEEL = 7;
        public static final int MOTOR_LOADER = 8;
        public static final int MOTOR_ANGLER = 9;
        public static final int MOTOR_SHOOTER = 10;
        public static final int MOTOR_LEVELER = 11;
    
        public static final int LEVEL_MOTOR_ID = 12;
        

    }

    public static final class JoystickConstants {
        // Deadbands ignore smaller inputs from joystick for smoother driving
        public static final double DEADBAND = 0.2;

        // Sets JoyStick Sensitivity (Even Numbers ONLY)
        public static double JOYSTICK_SENSITIVITY = 2;

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


		public static int LEVEL_MOTOR_ID;
    }

    public static final class LimeLightConstants {
        // PipeLine ID of Shooter
        public static final int SHOOTER_PIPELINE = 0;

        // Holds Proportional Constant for Aiming
        public static final double AIM_P = 1;

        // Holds the Minium Power to Move Robot
        public static final double AIM_MIN_COMMAND = 0.1;
    }

    public static final class LightStripConstants {
        public static final int PWM_PORT = 9;
        public static final int NUM_PIXELS = 60;
        
    }

}
