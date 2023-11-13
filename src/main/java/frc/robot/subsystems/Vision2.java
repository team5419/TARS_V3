// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.limelight.LimelightHelpers;
// import frc.robot.subsystems.limelight.LimelightHelpers.LimelightResults;

// public class Vision2 extends SubsystemBase {
// 	Swerve swerve;
	
// 	LimelightResults prevResult;
// 	public boolean isLimelightAlive = true;

// 	public Notifier limelightOfflineUpdater = new Notifier(() -> {
// 		LimelightResults current = LimelightHelpers.getLatestResults("");
// 		isLimelightAlive = prevResult == null ? false : (prevResult != current);
// 		prevResult = current;
// 	});

// 	public Vision2 (Swerve swerve) {
// 		this.swerve = swerve;
// 		limelightOfflineUpdater.startPeriodic(0.4);
// 	}

// 	public void periodic() {
// 		// Pose2d llPose = DriverStation.getAlliance() == Alliance.Blue ? LimelightHelpers.getBotPose2d_wpiBlue("") : LimelightHelpers.getBotPose2d_wpiRed("");
// 		// swerve.estimator.addVisionMeasurement(llPose, Timer.getFPGATimestamp());		
// 	}


// 	public boolean limelightSeesTarget() {
// 		return LimelightHelpers.getTV("");
// 	}

// 	public void enableLEDs() {
// 		LimelightHelpers.setLEDMode_ForceOn("");
// 	}

// 	public void disableLEDs() {
// 		LimelightHelpers.setLEDMode_ForceOff("");
// 	}

// 	public int getId () {
// 		return (int)LimelightHelpers.getFiducialID("");
// 	}

// 	public double[] getRobotPoseInFieldSpace() {
// 		return LimelightHelpers.getBotPose("");
// 	}

// 	public double[] getBotposeColorRelative (boolean isBlue) {
// 		return isBlue ? LimelightHelpers.getBotPose_wpiBlue("") : LimelightHelpers.getBotPose_wpiRed("");
// 	}

// 	public double[] getTargetInRobotSpace () {
// 		return LimelightHelpers.getTargetPose_RobotSpace("");
// 	}

// 	public double[] getRobotInTargetSpace () {
// 		return LimelightHelpers.getBotPose_TargetSpace("");
// 	}
// }
