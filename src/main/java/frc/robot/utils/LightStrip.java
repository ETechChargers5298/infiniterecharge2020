package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

//Creating the class, "LightStrip."
public class LightStrip {
    
    /* LIGHTSTRIP FIELDS */
    public static AddressableLED ledStrip;
    public static AddressableLEDBuffer ledStripBuffer;
    public static int numPixels = Constants.LightStripConstants.NUM_PIXELS;
    
    /* LIGHTSTRIP CONSTRUCTOR */
    public LightStrip()
    {

        ledStrip = new AddressableLED(Constants.LightStripConstants.PWM_PORT);
        ledStripBuffer = new AddressableLEDBuffer(numPixels);
        
        ledStrip.setLength(numPixels);
        ledStrip.setData(ledStripBuffer);
        ledStrip.start();
    }

    /* LIGHTSTRIP METHODS */
    public void setRGBValue(int r, int g, int b) {
     
        for (var i = 0; i < numPixels; i++) {
        ledStripBuffer.setRGB(i, r, g, b); //Sets RGB values.
        }  
        ledStrip.setData(ledStripBuffer);
    }

    public void setHSVValue(int h, int s, int v) {

        for (var i = 0; i < numPixels; i++) {
        ledStripBuffer.setHSV(i, h, s, v); //Sets HSV values.
        }
        ledStrip.setData(ledStripBuffer);
    }

    public void rainbow() {

        int rainbowFirstPixelHue = 0;

        for (var i = 0; i < numPixels; i++) {
            final var hue = (rainbowFirstPixelHue + (i * 180 / numPixels)) % 180;
            ledStripBuffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue =+ 3;
        rainbowFirstPixelHue %= 180;
        
        ledStrip.setData(ledStripBuffer);
    }
    
}