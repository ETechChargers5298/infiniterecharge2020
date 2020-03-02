package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimeLightConstants;

public class LimeLight {

    /*LIMELIGHT FIELDS */

    // Table to Obtain Values from LimeLight
    private NetworkTable table;
    
    // If Target is Aquired
    private double target;

    // Coordinates of Target
    private double x;
    private double y;

    // Area of Target Compared to Camera View Size
    private double area;

    // Boolean to Check if LimeLight has Valid Target
    private boolean hasValidTarget;


    /* LIMELIGHT CONSTRUCTOR */
    public LimeLight() {
        // Passes LimeLight's Network Table
        table = NetworkTableInstance.getDefault().getTable("limelight");

        // Gets Data From LimeLight
        target = table.getEntry("tv").getDouble(0);
        x = table.getEntry("tx").getDouble(0);
        y = table.getEntry("ty").getDouble(0);
        area = table.getEntry("ta").getDouble(0);

        // The Robot Should Start Off with No Target
        hasValidTarget = false;
    }

    /*LIMELIGHT METHODS*/
    
    // Updates Fields LimeLight Values
    public void updateLimeLight() {
        // Takes Data from LimeLight Network Tables
        target = table.getEntry("tv").getDouble(0);
        x = table.getEntry("tx").getDouble(0);
        y = table.getEntry("ty").getDouble(0);
        area = table.getEntry("ta").getDouble(0);

        // Checks if Valid Target is Obtained
        hasValidTarget = (target >= 1.0);
    }
    
    // Prints LimeLight Data
    public void printData() {
        // Prints Coordinates of Target
        SmartDashboard.putNumber("LimeLight X", x);
        SmartDashboard.putNumber("LimeLight Y", y);

        // Prints If Target has Been Obtained
        SmartDashboard.putBoolean("Target Found", hasValidTarget);

        // Prints Area of Target
        SmartDashboard.putNumber("Area of Target", area);
    }

    public void calibrate(double x, double y) {
        table.getEntry("cx0").setNumber(x);
        table.getEntry("cy0").setNumber(y);
    }

    // Getter for if Target is Obtained
    public boolean hasValidTarget() {
        return hasValidTarget;
    }

    // Getter for X Coordinate
    public double getX() {
        return x;
    }

    // Getter for Y Coordinate
    public double getY() {
        return y;
    }

    // Getter for Area of Target
    public double getArea() {
        return area;
    }

    // Switch to Shooter Pipeline
    public void shootPipe() {
        table.getEntry("pipeline").setNumber(LimeLightConstants.SHOOTER_PIPELINE);
    }

    // Switches to Driver Camera for Driver View
    public void driveCamera() {
        table.getEntry("camMode").setNumber(1);
    }

    // Switches to Vision Processing
    public void visionProcessor() {
        table.getEntry("camMode").setNumber(0);
    }

    // Uses PipeLine Default LEDs
    public void lightDefault() {
        table.getEntry("ledMode").setNumber(0);
    }

    // Turns LEDs off
    public void lightOff() {
        table.getEntry("ledMode").setNumber(1);
    }

    // Makes LEDs Blink
    public void lightBlink() {
        table.getEntry("ledMode").setNumber(2);
    }
    
    // Turns LEDs on
    public void lightOn() {
        table.getEntry("ledMode").setNumber(3);
    }
}