package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.arm.Waypoint;
import java.util.LinkedList;

/**
 * @author Grayson / Ben
 */
public class OptimizedMoveV3 extends CommandBase {

  private final OptimizedArm arm;
  private ArmTargets target;
  private LinkedList<ArmState> waypoints;

  GenericEntry velocityLogger;
  GenericEntry cleLogger;
  GenericEntry voltageLogger;
  GenericEntry currentLogger;

  WPI_TalonFX wrist;

  public OptimizedMoveV3(OptimizedArm arm, ArmTargets target) {
    this.arm = arm;
    this.target = target;

    wrist = arm.getWristTalon();

    // velocityLogger = Shuffleboard.getTab("Tuning").add("Velocity Wrist",
    // 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
    // cleLogger = Shuffleboard.getTab("Tuning").add("Closed Loop Error",
    // 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
    // voltageLogger = Shuffleboard.getTab("Tuning").add("Voltage",
    // 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
    // currentLogger = Shuffleboard.getTab("Tuning").add("Current",
    // 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.stop(); // Stop the arm
    arm.resetMotionMagic(); // Reset the motion magic

    waypoints = new LinkedList<ArmState>();

    ArmWaypoints[] tracePoints =
        GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()), target);

    for (ArmWaypoints point : tracePoints) {
      waypoints.add(point.point);
    }

    waypoints.add(
        new ArmState(
            OptimizedArm.ticksToDegreesBicep(target.bicepTarget),
            OptimizedArm.ticksToDegreesWrist(target.wristTarget)));

    if (tracePoints.length != 0 && tracePoints[0] == null) {
      System.err.println("[OPTIMIZED MOVE] Invalid position requested, abandoning move");
      this.cancel();
      return;
    } else {
      configAndMoveTo(waypoints.getFirst());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("Wrist - Vel", wrist.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Wrist - CLE", wrist.getClosedLoopError());
    // SmartDashboard.putNumber("Wrist - OUT VOLT", wrist.getMotorOutputVoltage());
    // SmartDashboard.putNumber("Wrist - OUT CURRENT", wrist.getStatorCurrent());
    SmartDashboard.putNumber(
        "Wrist - CURRENT DEG", OptimizedArm.ticksToDegreesWrist(wrist.getSelectedSensorPosition()));
    SmartDashboard.putNumber(
        "Wrist - SETPOINT DEG", OptimizedArm.ticksToDegreesWrist(wrist.getClosedLoopTarget()));

    if (arm.isAtWithEpsilon(waypoints.getFirst(), 2000)) {
      waypoints.removeFirst();
      if (waypoints.isEmpty()) return;
      configAndMoveTo(waypoints.getFirst());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      arm.stop();
    }

    arm.resetMotionMagic();

    System.out.println(
        "EXITED -------------------------------------------------------------------------------------------");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return waypoints.isEmpty();
  }

  private void configAndMoveTo(ArmState state) {

    System.out.println("Going to waypoint: " + state.bicep + ", " + state.wrist);

    arm.configMotionMagic(
        GraphStator.calculateNewMotionMagic(
            new Waypoint(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
            new Waypoint(
                OptimizedArm.degreesToTicksBicep(state.bicep),
                OptimizedArm.degreesToTicksWrist(state.wrist)),
            arm,
            true));

    arm.setBicep(ControlMode.MotionMagic, OptimizedArm.degreesToTicksBicep(state.bicep));
    arm.setWrist(ControlMode.MotionMagic, OptimizedArm.degreesToTicksWrist(state.wrist));
  }

  // private void configAndMoveTo (Waypoint point) {
  //     arm.configMotionMagic(
  //                 GraphStator.calculateNewMotionMagic(
  //                     new Waypoint(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
  //                     new Waypoint(point.bicep, point.wrist),
  //                     arm, true
  //                 )
  //             );

  //     arm.setBicep(ControlMode.MotionMagic, point.bicep);
  //     arm.setWrist(ControlMode.MotionMagic, point.wrist);
  // }

}
