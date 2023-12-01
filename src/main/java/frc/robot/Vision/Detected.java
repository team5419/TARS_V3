package frc.robot.Vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import java.util.Objects;

//amazing use case for records in jdk14...
public class Detected {
    private NetworkTableEntry label;
    private NetworkTableEntry certainty;
    private NetworkTableEntry yBottom;
    private NetworkTableEntry yTop;
    private NetworkTableEntry xLeft;
    private NetworkTableEntry xRight;

    public Detected(NetworkTableEntry label, NetworkTableEntry certainty, NetworkTableEntry yBottom, NetworkTableEntry yTop, NetworkTableEntry xLeft, NetworkTableEntry xRight){
        this.label = label;
        this.certainty = certainty;
        this.yBottom = yBottom;
        this.yTop = yTop;
        this.xLeft = xLeft;
        this.xRight = xRight;
    }

    public NetworkTableEntry getLabel() {
        return this.label;
    }

    public NetworkTableEntry getCertainty() {
        return this.certainty;
    }

    public NetworkTableEntry getYBottom() {
        return this.yBottom;
    }

    public NetworkTableEntry getYTop() {
        return this.yTop;
    }

    public NetworkTableEntry getXLeft() {
        return this.xLeft;
    }

    public NetworkTableEntry getXRight() {
        return this.xRight;
    }


}
