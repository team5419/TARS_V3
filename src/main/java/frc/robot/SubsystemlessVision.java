package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightHelpers.LimelightResults;

public class SubsystemlessVision {

  private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry validTarget;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private ShuffleboardTab tab = Shuffleboard.getTab("ll");
  private boolean alive = true;
  private double[] rbootpose_array = new double[6];
  private Notifier limelightStatusNotifier =
      new Notifier(
          () -> {
            LimelightResults current = LimelightHelpers.getLatestResults("limelight");
            alive = current != null;
          });

  public SubsystemlessVision() {
    configLL();
    limelightStatusNotifier.startPeriodic(0.25);
    updateEntries();
    tab.addDouble("0", () -> rbootpose_array[0]);
    tab.addDouble("1", () -> rbootpose_array[1]);
    tab.addDouble("2", () -> rbootpose_array[2]);
    tab.addDouble("3", () -> rbootpose_array[3]);
    tab.addDouble("4", () -> rbootpose_array[4]);
    tab.addDouble("5", () -> rbootpose_array[5]);
  }

  public void configLL() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  // call in 'some' periodic
  public void updateEntries() {
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    rbootpose_array = limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    // DEFAULT VALUE

  }

  public boolean isAlive() {
    return alive;
  }

  public boolean targetPresent() {
    return (limelight.getEntry("tv").getInteger(0) == 0);
  }

  public int getID() {
    return (int) limelight.getEntry("tid").getInteger(-1);
  }

  public double[] getTargetInRobotSpace() {

    rbootpose_array = limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

    return limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  public double[] getRobotInTargetSpace() {
    return limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  }

  public double[] getBotposeColorRelative(boolean isBlue) {
    return isBlue
        ? limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6])
        : limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
  }

  public double getTX() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  public double getTY() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  public double getHorizontalDistance() {
    return rbootpose_array[0];
  }

  public double getVertical() {
    return rbootpose_array[2];
  }
}
