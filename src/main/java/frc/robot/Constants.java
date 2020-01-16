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
    // Holds CAN IDs from SparkMax speed controllers
    public static final int MOTOR_LEFT_ZERO = 6;
    public static final int MOTOR_LEFT_ONE = 5;
    public static final int MOTOR_LEFT_TWO = 4;
    public static final int MOTOR_RIGHT_ZERO = 1;
    public static final int MOTOR_RIGHT_ONE = 2;
    public static final int MOTOR_RIGHT_TWO = 3;

    // Holds wheel inversion boolean (One should be true and the other one should be false)
    public static final boolean LEFT_INVERSION = true;
    public static final boolean RIGHT_INVERSION = false;

    // Constant that is multiplied to power to lower max possible power
    public static final double SPEED_MULTIPLIER = 1.0;

    // Deadbands ignore smaller inputs from joystick for smoother driving
    public static final double DEADBAND = 0.2;

    public static final int DRIVECONTROLLER = 1;
    public static final int OPERATORCONTROLLER = 2;
}
