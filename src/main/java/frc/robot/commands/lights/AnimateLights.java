package frc.robot.commands.lights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AnimateLights extends CommandBase {

    private Lights leds;
    private Swerve swerve;
    private Intake intake;
    
    private int brightness = 0;
    private boolean isGoingUp = true;
    private int changeRate = 5;

    // variables for rainbow animation
    int m_rainbowFirstPixelHue = 0; // red
    public int hueShiftStep = 1;

    public AnimateLights(Lights lights, Swerve swerve, Intake intake) {
        addRequirements(lights);
        leds = lights;
        this.swerve = swerve;
        this.intake = intake;
    }

    public void initialize() {
        
    }

    public void execute() {        
        if(brightness <= 255 && isGoingUp) {
            brightness += changeRate;
        } else if (!isGoingUp) {
            brightness -= changeRate;
        }

        if(brightness == 255) {
            isGoingUp = false;
        }

        if (brightness <= 50) {
            isGoingUp = true;
        }

        if (leds.debugMode || leds.overrideColor) {
            return;
        }

        if(DriverStation.isEStopped()) {
            leds.strobe(255, 255, 255, 1);
        } else if (intake.getIsStalling() && swerve.isIntakeActive) {
            leds.strobe(0, 255, 0, 0.5);
        } else if (swerve.isIntakeActive) {
            leds.strobe(255, 0, 0, 0.5);
        } else if (leds.enableRainbow) {
            RainbowTick();
        } else if(swerve.isUsingCones) {
            leds.setColor(brightness, (int)(95 * (brightness / 255.0)), 0);
        } else if (!swerve.isUsingCones) {
            leds.setColor(brightness, 0, brightness);
        } else {
            leds.setColor(brightness, 0, 0);
        }
    }

    public void end(boolean interrupted) {

        leds.setColor(0, 0, 0);
    }

    public boolean isFinished() {
        return false;
    }

    // from https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#creating-a-rainbow-effect
    private void RainbowTick() {
        final AddressableLEDBuffer buffer = new AddressableLEDBuffer(leds.ledLength);
         // For every pixel
        for (var i = 0; i < leds.ledLength; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / leds.ledLength)) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;

        leds.setColor(buffer);
    }
}
