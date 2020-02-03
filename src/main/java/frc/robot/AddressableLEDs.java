package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

//Creating the class, "AddressableLEDs."
public class AddressableLEDs {
    
    public static AddressableLED ledStrip;
    public static AddressableLEDBuffer ledStripBuffer; 
    
    public AddressableLEDs()
    {

        ledStrip = new AddressableLED(9);
        
        ledStripBuffer = new AddressableLEDBuffer(60);
        
        ledStrip.setLength(ledStripBuffer.getLength());

        ledStrip.setData(ledStripBuffer);
        ledStrip.start();
    }

    public void setRGBValue(int r, int g, int b) {
     
        for (var i = 0; i < ledStripBuffer.getLength(); i++) {
        ledStripBuffer.setRGB(i, r, g, b); //Sets RGB values.
        }  
        ledStrip.setData(ledStripBuffer);
    }

    public void setHSVValue(int h, int s, int v) {

        for (var i = 0; i < ledStripBuffer.getLength(); i++) {
        ledStripBuffer.setHSV(i, h, s, v); //Sets HSV values.
        }
        ledStrip.setData(ledStripBuffer);
    }

    public void rainbow() {

        int rainbowFirstPixelHue = 0;

        for (var i = 0; i < ledStripBuffer.getLength(); i++) {
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledStripBuffer.getLength())) % 180;
            ledStripBuffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue =+ 3;
        rainbowFirstPixelHue %= 180;
        
        ledStrip.setData(ledStripBuffer);
    }
    
}