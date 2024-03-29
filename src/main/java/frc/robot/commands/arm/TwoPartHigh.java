package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;
import java.util.function.BooleanSupplier;

/**
 * @author Grayson
 */
public class TwoPartHigh extends SequentialCommandGroup {
  public TwoPartHigh(
      OptimizedArm arm, ArmTargets target, boolean isCone, BooleanSupplier shootSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    // ArmTargets start = new ArmTargets(arm.getBicepPosition(), arm.getWristPosition());
    ArmTargets customIntermediaryTarget =
        new ArmTargets(target.bicepTarget + OptimizedArm.degreesToTicksBicep(15), 0);

    ArmWaypoints[] waypoints =
        GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
            customIntermediaryTarget);

    if (waypoints.length != 0 && waypoints[0] == null) {
      System.out.println("[TWO PART HIGH] - Invalid position requested, abandoning request");
      return;
    }

    for (ArmWaypoints point : waypoints) {
      addCommands(new RetimeArm(arm, point, false), new ParallelToPos(arm, point, false));
    }

    addCommands(
        new WaitUntilCommand(shootSupplier),
        new WaitCommand(0.5),
        new RetimeArm(arm, customIntermediaryTarget, false),
        new ParallelToPos(arm, customIntermediaryTarget, false),
        new RetimeArm(arm, Constants.ArmConstants.stow, true),
        new ParallelToPos(arm, Constants.ArmConstants.stow, true));
  }
}
