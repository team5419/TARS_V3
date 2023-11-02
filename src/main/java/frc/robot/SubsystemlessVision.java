package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
/*
 * @author Ammel
 */
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightHelpers.LimelightResults;

public class SubsystemlessVision {

  // private final NetworkTable limelight = NetworkTableInstance
  //   .getDefault()
  //   .getTable("limelight");
  // private NetworkTableEntry validTarget, tx, ty;
  // private ShuffleboardTab tab = Shuffleboard.getTab("limelight");
  // private boolean alive = true;
  // private Notifier limelightStatusNotifier = new Notifier(() -> {
  //   LimelightResults current = LimelightHelpers.getLatestResults("");
  //   alive = current != null;
  // });

  public SubsystemlessVision() {
    // limelightStatusNotifier.startPeriodic(0.25);
  }

  // call in 'some' periodic
  // public void updateEntries() {
  //   tx = limelight.getEntry("tx");
  //   ty = limelight.getEntry("ty");
  //   //DEFAULT VALUE
  //   tab.addDouble("LLX", () -> tx.getDouble(-100000000));
  //   tab.addDouble("LLY", () -> ty.getDouble(-100000000));
  // }

  // public boolean isAlive() {
  //   return alive;
  // }

  // public boolean targetPresent() {
  //   return (limelight.getEntry("tv").getInteger(0) == 0);
  // }

  // public int getID() {
  //   return (int) limelight.getEntry("tid").getInteger(-1);
  // }

  // public double[] getTargetInRobotSpace() {
  //   return limelight
  //     .getEntry("targetpose_robotspace")
  //     .getDoubleArray(new double[6]);
  // }

  // public double[] getRobotInTargetSpace() {
  //   return limelight
  //     .getEntry("botpose_targetspace")
  //     .getDoubleArray(new double[6]);
  // }

  // public double[] getBotposeColorRelative(boolean isBlue) {
  //   return isBlue
  //     ? limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6])
  //     : limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
  // }
  // public double getTX(){
  //   return limelight.getEntry("tx").getDouble(0.0); 
  // }
  // public double getTY(){
  //   return limelight.getEntry("ty").getDouble(0.0);
  // }
  // public double getHorizontalDistance(){
  //   return getTargetInRobotSpace()[1];
  // }
  //   public double getVertical(){
  //   return getTargetInRobotSpace()[2];
  // }
}
