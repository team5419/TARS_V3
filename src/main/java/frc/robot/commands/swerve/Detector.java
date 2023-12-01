package frc.robot.commands.swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightHelpers.LimelightResults;

public class Detector {

    private NetworkTable pi = NetworkTableInstance
    .getDefault()
    .getTable("detector");

    //ASSUMES ONE OBJECT
    private NetworkTableEntry label;
    private NetworkTableEntry certainty;
    private NetworkTableEntry yBottom;
    private NetworkTableEntry yTop;
    private NetworkTableEntry xLeft;
    private NetworkTableEntry xRight;

    private ShuffleboardTab tab = Shuffleboard.getTab("detector");
    private boolean alive = true;
    private Notifier limelightStatusNotifier = new Notifier(() -> {
    //!! Alter to reflect new architecture !!// 
    // LimelightResults current = LimelightHelpers.getLatestResults("limelight");
    // alive = current != null;
    });

    public Detector(){
        // limelightStatusNotifier.startPeriodic(0.25);
        configTab();
    }
    private void configTab(){
        tab.addString("Label: ", () -> label.getString("NULL"));
        tab.addDouble("Y BOTTOM: ", () -> yBottom.getDouble(0.0));
        tab.addDouble("Y TOP: ", () -> yTop.getDouble(0.0));
        tab.addDouble("X LEFT: ", () -> xLeft.getDouble(0.0));
        tab.addDouble("X RIGHT: ", () -> xRight.getDouble(0.0));
        tab.addDouble("CERTAINTY", certainty.getDouble(0.0));`
    }

    public void updateEntries(){
        label = pi.getEntry("Label");
        certainty = pi.getEntry("Certainty");
        yBottom = pi.getEntry("Y Bottom");
        yTop = pi.getEntry("Y Top");
        xLeft = pi.getEntry("X Left");
        xRight = pi.getEntry("X Right");

    }
    public boolean isAlive() {
        return alive;
    }
}