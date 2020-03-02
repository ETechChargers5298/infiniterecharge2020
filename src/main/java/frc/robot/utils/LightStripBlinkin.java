package frc.robot.utils;

import edu.wpi.first.wpilibj.Spark;

public class LightStripBlinkin {
    // Holds REV Blinken
    private Spark controller;
    
    public LightStripBlinkin(int port) {
        // Uses Blinken as a Speed Controller
        controller = new Spark(port);
    }

    public void solidRed() {
        controller.setSpeed(0.61);
    }

    public void solidBlue() {
        controller.setSpeed(0.87);
    }

    public void confetti() {
        controller.setSpeed(-0.87);
    }

    public void rainbow() {
        controller.setSpeed(-0.99);
    }

    //This color is for when the robot ISN'T level on the generator switch.
    public void twinklesLava() {
        controller.setSpeed(-0.49);
    }

    //This color is for when the robot IS level on the generator switch.
    public void twinklesOcean() {
        controller.setSpeed(-0.51);
    }

    //This color is for when the robot is MOVING.
    public void solidGreen() {
        controller.setSpeed(-0.77);
    }

    //This color is for when the robot is STILL.
    public void solidYellow() {
        controller.setSpeed(0.69);
    }

    //This color is for when the robot is UNDER the optimal shooting level.
    public void sinelonOcean() {
        controller.setSpeed(-0.75);
    }

    //This color is for when the robot is OVER the optimal shooting level.
    public void sinelonLava() {
        controller.setSpeed(-0.73);
    }

    //This color is for when the robot is AT the optimal shooting level.
    public void sinelonForest() {
        controller.setSpeed(-0.71);
    }

    //This color is for when the intake is UP.
    public void solidWhite() {
        controller.setSpeed(0.93);
    }
}