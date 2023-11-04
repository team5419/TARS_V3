package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.SubsystemlessVision;
import frc.robot.Util;
import frc.robot.autos.AutoHelpers.Point;
import frc.robot.autos.CustomPath;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.limelight.LimelightHelpers;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * @author Ammel
 */
public class AutoAlignPenn extends CommandBase {

  private final Swerve swerve;
  private final OptimizedArm arm;
  //test true one more time, just to see with points being made correct
  private final boolean custom = false;
  private final SubsystemlessVision vision = new SubsystemlessVision();
  private CustomPath path;
  private final PIDController horizontalController;
  private final PIDController veritcalController;
  private final ProfiledPIDController rotationController;
  private final double epsilonDistance = 1;
  private double distanceToTarget;
  private Timer timer;
  private double timeLimit;
  private ShuffleboardTab tab = Shuffleboard.getTab("AUTOALIGN - PENN");
  private Command currentCommand;

  private double newHorizontal = 5.0;
  private double newVertical = 5.0;
  private double newRotation = 5.0;

  public AutoAlignPenn(Swerve swerve, OptimizedArm arm, double timeLimit) {
    configTab();
    this.swerve = swerve;
    this.arm = arm;
    this.path = directPath();
    this.horizontalController = new PIDController(0.1, 0, 0); //untuned
    this.veritcalController = new PIDController(0.1, 0, 0); //untuned

    this.rotationController =
      new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0,
        Constants.AutoConstants.kThetaControllerConstraints
      );
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    this.timeLimit = timeLimit;
    this.timer = new Timer();
    addRequirements(swerve, arm);
  }

  public void configTab(){
    tab.addDouble("newHorizontal", () -> {return newHorizontal;});
    tab.addDouble("newVertical", () -> {return newVertical;});
    tab.addDouble("newRotation", () -> {return newRotation;});
    tab.addBoolean("custom", () -> {return custom;});
    tab.addDouble("tx", () -> {return vision.getTX();});
    tab.addDouble("ty", () -> {return vision.getTY();});
    tab.addDouble("DTT", () -> {return distanceToTarget;});

  }
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    vision.updateEntries();
    path = directPath();
    if(custom){
        customClosedLoopPathCalculation();
    } else{
      customTwo();
    }
    // else{
    //     System.out.println("CUSTOM");
    //     currentCommand = closedLoopPathCalculation();
    //     currentCommand.schedule();
    // }
    distanceToTarget =
      Util.getDistance(vision.getHorizontalDistance(), vision.getVertical());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    System.out.println("Terminating Auto Align");
  }

  @Override
  public boolean isFinished() {
    if(!vision.isAlive()){
      System.out.println("VISION NOT ALIVE");
    }
    if(distanceToTarget < epsilonDistance){
      System.out.println("DISTANCE TO TARGET LITTLE");
    }
    if(timer.get() > timeLimit){
      System.out.println("TIME LIMIT EXCEEDED");
    }
    return (
      !vision.isAlive() ||
      distanceToTarget < epsilonDistance ||
      timer.get() > timeLimit
    );
  }

  //shouldnt be void in earnest - in ref to customClosedPathCalculation
  //some stuff can go in swerve drive class ?
  //could i just use TrajectoryGeneartor


  //!! INCOMPLETE !!
  private Command closedLoopPathCalculation() {
    List<Pose2d> waypoints = new ArrayList<>();
    path.getPoints().forEach(point -> waypoints.add(Util.pointToPose2d(point)));
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared
    )
      .setKinematics(Constants.Swerve.swerveKinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      waypoints,
      trajectoryConfig
    );
    //! ^^^ We have path planner, an should probably be using that as it allows for an easier time.
    //* For how to use I'd take a look at the last constructor in the Auto.java file -- G

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory,
      swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPXController, 0, 0),
      new PIDController(Constants.AutoConstants.kPYController, 0, 0),
      rotationController,
      swerve::setModuleStates,
      swerve
    );
    return swerveControllerCommand;
  }

  private void customClosedLoopPathCalculation() {
    Point target;
    Point current = path.current();
    //arbitrary constants
    if (
      Util.getDistance(current.x, current.y) < 0.1 &&
      (swerve.getRotationDegrees() - current.rotation) < 5
    ) {
      target = path.next();
      System.out.println("NEXT POINT IN PATH");
    } else {
      target = current;
    }
    //possibly swapped
    newHorizontal = horizontalController.calculate(
      vision.getTX(),
      target.x
    );
    newVertical = veritcalController.calculate(vision.getTY(), target.y);
    newRotation = rotationController.calculate(
      swerve.getRotationDegrees(),
      target.rotation
    );
    //??Why not fieldrelative or closed loop? Possibly something to fiddle with...
    swerve.drive(
      new Translation2d(newHorizontal, newVertical),
      newRotation,
      false,
      false
    );
    System.out.println("Target Y " +target.y);
    System.out.println("TARGET X " +target.x);
  }

  // private CustomPath customPath() {
  //   Point[] pointsArray = { new Point(0, 0, 0), new Point(0, 0, 0) };
  //   List<Point> points = new ArrayList<>();
  //   points.addAll(Arrays.asList(pointsArray));
  //   return new CustomPath(points);
  // }

  private CustomPath directPath() {
    List<Point> points = new ArrayList<>();
    //not sure if array indexes are correct
    points.add(
      new Point(vision.getHorizontalDistance(), vision.getVertical(), 0)
    );
    return new CustomPath(points);
  }
private Point getTarget(){
    return new Point(vision.getHorizontalDistance(), vision.getVertical(), 0);
}
public void customTwo(){
   Point target = getTarget();
   System.out.println(target);

    newHorizontal = horizontalController.calculate(
      0.0,
      target.x
    );
    newVertical = veritcalController.calculate(
      0.0, 
      target.y
    );
    newRotation = rotationController.calculate(
      swerve.getRotationDegrees(),
      target.rotation
    );
    swerve.drive(
      new Translation2d(newHorizontal, newVertical),
      newRotation,
      false,
      false
    );
}
}


//Get Position of Tag In Robot Space
//Drive To Tag Position