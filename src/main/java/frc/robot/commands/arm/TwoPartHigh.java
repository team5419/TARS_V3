package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import javax.swing.GroupLayout.SequentialGroup;

import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;

/**
 * @author Grayson
 */
public class TwoPartHigh extends SequentialCommandGroup {
    public TwoPartHigh(OptimizedArm arm, ArmTargets target, boolean isCone, BooleanSupplier shootSupplier) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);

        // ArmTargets start = new ArmTargets(arm.getBicepPosition(), arm.getWristPosition());
        ArmTargets customIntermediaryTarget = new ArmTargets(
            target.bicepTarget + OptimizedArm.degreesToTicksBicep(15), 
            0
        );

        ArmWaypoints[] waypoints = GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
            target
        );

        if(waypoints.length == 0) {
            addCommands(
                new RetimeArm(arm, customIntermediaryTarget),
                new ParallelToPos(arm, customIntermediaryTarget, false),
                new RetimeArm(arm, target),
                new ParallelToPos(arm, target, true)
            );
        } else if (waypoints[0] == null) {
            System.out.println("[TWO PART HIGH] - Invalid position requested, abandoning request");
            return;
        } else if (waypoints.length == 1) {
            addCommands(
                new RetimeArm(arm, waypoints[0]),
                new ParallelToPos(arm, waypoints[0], false),
                new RetimeArm(arm, customIntermediaryTarget),
                new ParallelToPos(arm, customIntermediaryTarget, false),
                new RetimeArm(arm, target),
                new ParallelToPos(arm, target, true)
            );
        } else {
            addCommands(
                new RetimeArm(arm, waypoints[0]),
                new ParallelToPos(arm, waypoints[0], false),
                new RetimeArm(arm, waypoints[1]),
                new ParallelToPos(arm, waypoints[1], false),
                new RetimeArm(arm, customIntermediaryTarget),
                new ParallelToPos(arm, customIntermediaryTarget, false),
                new RetimeArm(arm, ArmWaypoints.QUAD_E),
                new ParallelToPos(arm, ArmWaypoints.QUAD_E, true),
                new RetimeArm(arm, target),
                new ParallelToPos(arm, target, true)
            );
        }

        addCommands(
            new WaitUntilCommand(shootSupplier),
            new WaitCommand(0.5),
            new RetimeArm(arm, customIntermediaryTarget),
            new ParallelToPos(arm, customIntermediaryTarget, false),
            new RetimeArm(arm, Constants.ArmConstants.stow),
            new ParallelToPos(arm, Constants.ArmConstants.stow, true)
        );
    }
}