package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision2 extends SubsystemBase {
	NetworkTable limelight = null;
	NetworkTableEntry validTargets, tx, ty;
	
	NetworkTable previousTable;
	public boolean isLimelightAlive = true;

	// public Notifier limelightOfflineUpdater = new Notifier(() -> {
	// 	if(previousTable != null) {
	// 		if(previousTable == limelight) {
	// 			isLimelightAlive = false;
	// 		} else {
	// 			isLimelightAlive = true;
	// 		}
	// 	} 

	// 	previousTable = limelight;
	// });

	public Vision2 () {
		// limelight = NetworkTableInstance.getDefault().getTable("limelight");
		// limelight.getEntry("pipeline").setNumber(1); // Put us in apriltags mode for now

		// try {
		// 	aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
		// } catch (Exception e) {
		// 	System.err.println("Apriltag field positioning could not be initialized. Full trace: " + e);
		// 	Logger.getInstance().recordOutput("Vision 2", "Apriltag field positioning could not be initialized. Full trace: " + e);
		// 	System.exit(1000);
		// }

		// Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, Constants.PhotonConstants.cameraHeight), new Rotation3d(0, 0, 0));
		// photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);

		// limelightOfflineUpdater.startPeriodic(0.1);
	}

	public void periodic() {

		tx = limelight.getEntry("tx"); // horizontal offset
		ty = limelight.getEntry("ty"); // horizontal offset

		// if (isPhotonEstimating) {
		// 	Optional<EstimatedRobotPose> estimatedPose = photonEstimator.update();
		// 	if(estimatedPose.isPresent()) {
		// 		EstimatedRobotPose estimation = estimatedPose.get();
		// 		swerve.addVisionMeasurement(estimation.estimatedPose.toPose2d(), estimation.timestampSeconds, 2);
		// 	}
		// }

		// double[] limelightRaw = getBotposeColorRelative(DriverStation.getAlliance() == Alliance.Blue);
		// double[] limelightRaw = getBotposeColorRelative(true);
		// Pose2d limelightPose = new Pose2d(limelightRaw[0], limelightRaw[1], new Rotation3d(limelightRaw[2], limelightRaw[3], limelightRaw[4]).toRotation2d());
		// swerve.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp(), 0.01);		

	}

	// public boolean isLimelightAlive () {

	// }

	public boolean limelighSeesTarget() {
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

	// public Optional<PhotonPipelineResult> getPhotonResult() {
	// 	var result = camera.getLatestResult();
	// 	if(result.hasTargets()) {
	// 		return Optional.of(result);
	// 	}
	// 	return Optional.empty();
	// }

	// public double getDistanceToTarget(PhotonPipelineResult fromResult) {
	// 	return PhotonUtils.calculateDistanceToTargetMeters(
	// 		Constants.PhotonConstants.cameraHeight, 
	// 		Constants.PhotonConstants.nodeAprilTagHeight, 
	// 		Constants.PhotonConstants.cameraRotationRadians, 
	// 		Units.degreesToRadians(fromResult.getBestTarget().getPitch())
	// 	);
	// }

	// public Optional<Pose3d> getTagPose3d (int tagID) {
	// 	return aprilTagFieldLayout.getTagPose(tagID);
	// }
}
