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

    private NetworkTable limelight = NetworkTableInstance
    .getDefault()
    .getTable("limelight");
    private NetworkTableEntry tclass;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private ShuffleboardTab tab = Shuffleboard.getTab("detector");
    private boolean alive = true;
    private Notifier limelightStatusNotifier = new Notifier(() -> {
    LimelightResults current = LimelightHelpers.getLatestResults("limelight");
    alive = current != null;
    });

    public Detector(){
        limelightStatusNotifier.startPeriodic(0.25);
        configLL();
        configTab();
    }
    private void configTab(){
        tab.addString("Class: ", () -> tclass.getString("NULL"));
        tab.addDouble("TX: ", () -> tx.getDouble(0.0));
        tab.addDouble("TY: ", () -> ty.getDouble(0.0));
    }
    private void configLL(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }

    public void updateEntries(){
        tclass = limelight.getEntry("tclass");
        tx = limelight.getEntry("tx");
        ty = limelight.getEntry("ty");
    }
    public boolean isAlive() {
        return alive;
    }
    public String getTClass(){
        return tclass.getString("NULL");
    }
    public double getTX(){
        return tx.getDouble(0.0);
    }
    public double getTY(){
        return ty.getDouble(0.0);
    }
}