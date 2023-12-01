package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;

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

    private List<Detected> detected_objects;
    private NetworkTableEntry total_detected;

    private ShuffleboardTab tab = Shuffleboard.getTab("detector");
    private boolean alive = true;
    private Notifier limelightStatusNotifier = new Notifier(() -> {
    //!! Alter to reflect new architecture !!// 
    // LimelightResults current = LimelightHelpers.getLatestResults("limelight");
    // alive = current != null;
    });

    public Detector(){
        // limelightStatusNotifier.startPeriodic(0.25);
        // configTab();
    }
    private void configTab(){
    }

    public void updateEntries(){
        detected_objects = new ArrayList<>();
        total_detected = pi.getEntry("Total Objects Detected");
        for(var i = 0; i < total_detected.getInteger(0); i++){
            var label = pi.getEntry(i + "Label");
            var certainty = pi.getEntry(i + "Certainty");
            var yBottom = pi.getEntry(i + "Y Bottom");
            var yTop = pi.getEntry(i + "Y Top");
            var xLeft = pi.getEntry(i + "X Left");
            var xRight = pi.getEntry(i + "X Right");
            detected_objects.add(new Detected(label, certainty, yBottom, yTop, xLeft, xRight));
        updateShuffleboard();
        }

    }
    //TODO MAKE THIS
    public void updateShuffleboard(){
    }
    public List<Detected> getDetectedObjects(){
        return detected_objects;
    }
    public boolean isAlive() {
        return alive;
    }
}