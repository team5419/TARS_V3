package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.Ports;

public class Lights extends SubsystemBase {
    AddressableLED leds;
    AddressableLEDBuffer buffer;
    private int brightness;
    private boolean isGoingUp;
    private int changeRate;
    public boolean enableRainbow = false;

    public boolean debugMode = false;
    // for if you want to debug something, and don't need the current leds.

    public boolean isEnabled = false;
    public boolean overrideColor = false;

    // todo: make this read-only 
    public int ledLength = 52;
        
    
    public Lights(int ledLength) {
        this.ledLength = ledLength;
        leds = new AddressableLED(9);
        leds.setLength(ledLength);
        buffer = new AddressableLEDBuffer(ledLength);

        leds.start();
        System.out.println("LED STREAM STARTED");

        brightness = 255;
        isGoingUp = false;
        changeRate = 5;
        enableRainbow = false;
    }

    

    public void setColor(int r, int g, int b) {
        for(int i = 0; i < ledLength; i++) {
            buffer.setRGB(i, r, g, b);
        }
        
        leds.setData(buffer);
    }

    public void setColor(AddressableLEDBuffer buffer) {
        leds.setData(buffer);
    }

    public void setSingleLedColor(int index, int r, int g, int b) {

        if (index < 0 || index > ledLength) {
            throw new IndexOutOfBoundsException(index);
        }

        buffer.setRGB(index, r, g, b);
        leds.setData(buffer);
    }

    public void strobe(int r, int g, int b, double duration) {
        boolean isOn = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        if(isOn) {
            setColor(r, g, b);
        } else {
            setColor(0, 0, 0);
        }
    }

    public void tick() {
        if(DriverStation.isEStopped()) {
            strobe(255, 255, 255, 1);
            return;
        }

        if (debugMode || overrideColor) {
            return;
        }

        if(brightness <= 255 && isGoingUp) {
            brightness += changeRate;
        } else if (!isGoingUp) {
            brightness -= changeRate;
        }

        if(brightness == 255) {
            isGoingUp = false;
        }

        if (brightness <= 0) {
            isGoingUp = true;
        }
    
        setColor(brightness, 0, 0);
    }

    public void rainbow() {
        for(int i = 0; i < 255; i++){
            for(int j = 0; j < 255; j++){
                for(int k =0; j <255;k++){
                    if(i == 254 && j == 254 && k==254){
                        i = 0;
                        j = 0;
                        k=0;
                    }
                    setColor(i, j, k);
                }
            }
        }
    }

    public void off() {
        setColor(0, 0, 0);
    }
}
