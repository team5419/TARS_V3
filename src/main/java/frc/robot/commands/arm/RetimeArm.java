package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.arm.Waypoint;

/**
 * @author
 */
public class RetimeArm extends CommandBase {

  private final OptimizedArm arm;
  private ArmTargets target;
  private ArmWaypoints targetWaypoint;
  private boolean isLastMove = true;

  public RetimeArm(OptimizedArm arm, ArmTargets target, boolean isLastMove) {
    this.arm = arm;
    this.target = target;
    this.targetWaypoint = null;
    this.isLastMove = isLastMove;
  }

  public RetimeArm(OptimizedArm arm, ArmWaypoints targetWaypoint, boolean isLastMove) {
    this.arm = arm;
    this.target = null;
    this.targetWaypoint = targetWaypoint;
    this.isLastMove = isLastMove;
  }

  @Override
  public void initialize() {
    Waypoint startWaypoint =
        new Waypoint(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees());
    if (target == null) {
      arm.configMotionMagic(
          GraphStator.calculateNewMotionMagic(
              startWaypoint,
              new Waypoint(targetWaypoint.point.bicep, targetWaypoint.point.wrist),
              arm,
              isLastMove));
    } else {
      Waypoint endWaypoint =
          new Waypoint(
              OptimizedArm.ticksToDegreesBicep(target.bicepTarget),
              OptimizedArm.ticksToDegreesWrist(target.wristTarget));
      arm.configMotionMagic(
          GraphStator.calculateNewMotionMagic(startWaypoint, endWaypoint, arm, isLastMove));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
