package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.subsystems.limelight.LimelightHelpers.LimelightResults;

public class Vision2 extends SubsystemBase {
	NetworkTable limelight = null;
	NetworkTableEntry validTargets, tx, ty;
	
	LimelightResults prevResult;
	public boolean isLimelightAlive = true;

	public Notifier limelightOfflineUpdater = new Notifier(() -> {
		LimelightResults current = LimelightHelpers.getLatestResults("");
		isLimelightAlive = prevResult == null ? false : (prevResult != current);
		prevResult = current;
	});

	public Vision2 () {
		limelight = NetworkTableInstance.getDefault().getTable("limelight");
		limelightOfflineUpdater.startPeriodic(0.4);
	}

	public void periodic() {

		tx = limelight.getEntry("tx"); // horizontal offset
		ty = limelight.getEntry("ty"); // horizontal offset


		// double[] limelightRaw = getBotposeColorRelative(DriverStation.getAlliance() == Alliance.Blue);
		// Pose2d limelightPose = new Pose2d(limelightRaw[0], limelightRaw[1], new Rotation3d(limelightRaw[2], limelightRaw[3], limelightRaw[4]).toRotation2d());
		// swerve.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp(), 0.01);		

	}


	public boolean limelightSeesTarget() {
		int in = 0;
		limelight.getEntry("tv").getInteger(in);
		return in == 0;
	}

	public void enableLEDs() {
		limelight.getEntry("ledMode").setNumber(3);
	}

	public void disableLEDs() {
		limelight.getEntry("ledMode").setNumber(1);
	}

	public int getId () {
		int res = -1;
		limelight.getEntry("tid").getInteger(res);
		return res;
	}

	public double[] getRobotPoseInFieldSpace() {
		return limelight.getEntry("botpose").getDoubleArray(new double[6]);
	}

	public double[] getBotposeColorRelative (boolean isBlue) {
		if(isBlue) {
			return limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		} else {
			return limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);

		}
	}

	public double[] getTargetInRobotSpace () {
		return limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
	}

	public double[] getRobotInTargetSpace () {
		return limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
	}
}
