// package frc.robot.commands.swerve;

// import java.util.Queue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Vision2;

// /* A faux auto ported from lithium the Snail */
// public class LimelightDriveToDistance extends CommandBase {

// 	private final Vision2 vision;
// 	private final Swerve swerve;

// 	private Timer timer = new Timer();
// 	private double timeoutTime = -1;

// 	// private double rotationFromTag = 0;
// 	private double distFromTag = 3;
// 	private double targetDist = 2.9;

// 	private PIDController controller = new PIDController(0.05, 0, 0);
// 	// private PIDController rotationController = new PIDController(0.1, 0, 0);
// 	private double epsilon = 0.05;
// 	private double maxSpeed = 1.2;
// 	private double speedMultiplier = 100;

// 	private boolean forceExit = false;

// 	private boolean shouldJostle = false;

// 	public LimelightDriveToDistance(Vision2 vision, Swerve swerve, double targetDist) {
// 		// Use addRequirements() here to declare subsystem dependencies.
// 		this.vision = vision;
// 		this.swerve = swerve;
// 		this.targetDist = targetDist;
// 		this.timeoutTime = -1;
// 		this.maxSpeed = 1.2;
// 		addRequirements(vision, swerve);
// 	}

// 	public LimelightDriveToDistance(Vision2 vision, Swerve swerve, double targetDist, double
// timeoutTime) {
// 		// Use addRequirements() here to declare subsystem dependencies.
// 		this.vision = vision;
// 		this.swerve = swerve;
// 		this.targetDist = targetDist;
// 		this.timeoutTime = timeoutTime;
// 		this.maxSpeed = 1.2;
// 		addRequirements(vision, swerve);
// 	}

// 	public LimelightDriveToDistance(Vision2 vision, Swerve swerve, double targetDist, double
// timeoutTime, double overrideMaxSpeed) {
// 		// Use addRequirements() here to declare subsystem dependencies.
// 		this.vision = vision;
// 		this.swerve = swerve;
// 		this.targetDist = targetDist;
// 		this.timeoutTime = timeoutTime;
// 		this.maxSpeed = overrideMaxSpeed;
// 		addRequirements(vision, swerve);
// 	}

// 		public LimelightDriveToDistance(Vision2 vision, Swerve swerve, double targetDist, double
// timeoutTime, double overrideMaxSpeed, boolean shouldJostleUponFailure) {
// 		// Use addRequirements() here to declare subsystem dependencies.
// 		this.vision = vision;
// 		this.swerve = swerve;
// 		this.targetDist = targetDist;
// 		this.timeoutTime = timeoutTime;
// 		this.maxSpeed = overrideMaxSpeed;
// 		shouldJostle = shouldJostleUponFailure;
// 		addRequirements(vision, swerve);
// 	}

// 	// Called when the command is initially scheduled.
// 	@Override
// 	public void initialize() {
// 		controller.reset();
// 		timer.restart();
// 	}

// 	// Called every time the scheduler runs while the command is scheduled.
// 	@Override
// 	public void execute() {
// 		distFromTag = vision.getTargetInRobotSpace()[2];
// 		// rotationFromTag = vision.getTargetInRobotSpace()[3];

// 		if(distFromTag > 5) {
// 			swerve.lock();
// 			forceExit = true;
// 			return;
// 		}

// 		double rotationSpeed = 0;
// 		// double currentRotation = swerve.getHeading().getDegrees();
// 		// if(vision.getTagPose3d(vision.getId()).isPresent()) {
// 		// 	rotationSpeed = -rotationController.calculate(currentRotation, (DriverStation.getAlliance()
// == Alliance.Blue ? 0 : 180));
// 		// }

// 		double driveSpeed = -controller.calculate(distFromTag, targetDist); // Limit the max speed

// 		if(driveSpeed > 0) {
// 			driveSpeed = Math.min(driveSpeed * speedMultiplier, maxSpeed); // make it actually do
// something
// 		} else if (driveSpeed < 0) {
// 			driveSpeed = Math.max(driveSpeed * speedMultiplier, -maxSpeed); // make it actually do
// something
// 		}

// 		if(!vision.limelightSeesTarget()) {
// 			forceExit = true;
// 			swerve.lock();
// 			return;
// 		}

// 		swerve.drive(new Translation2d(driveSpeed, 0), rotationSpeed, false, true);
// 	}

// 	// Called once the command ends or is interrupted.
// 	@Override
// 	public void end(boolean interrupted) {
// 		swerve.lock();
// 	}

// 	// Returns true when the command should end.
// 	@Override
// 	public boolean isFinished() {
// 		SwerveModuleState[] states = swerve.getModuleStates();

// 		double currentSpeed = (states[0].speedMetersPerSecond + states[1].speedMetersPerSecond +
// states[2].speedMetersPerSecond + states[3].speedMetersPerSecond) / 4.0; // average out all the
// speeds

// 		if(timeoutTime == -1) {
// 			return (MathUtil.applyDeadband((distFromTag - targetDist), epsilon) == 0) &&
// (MathUtil.applyDeadband(currentSpeed, 0.05) == 0) || forceExit;
// 		} else {
// 			return (MathUtil.applyDeadband((distFromTag - targetDist), epsilon) == 0) &&
// (MathUtil.applyDeadband(currentSpeed, 0.05) == 0) || (timer.get() >= timeoutTime) || forceExit;
// 		}
// 	}

// }
