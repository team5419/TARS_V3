package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.GraphStatorOLD;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.arm.Waypoint;

/**
 * @author Grayson
 */
public class OptimizedMove extends SequentialCommandGroup {
    public OptimizedMove(OptimizedArm arm, ArmTargets target) {
        // Get our stator, to calculate all the things that we need
        // GraphStatorOLD graphStator = arm.getGraphStator();

        // "trace" our path trough our imaginary graph to find all of our waypoints
        ArmWaypoints[] waypoints = GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
            target
        );

        // If we are in the same sector, then we are good to move wherever
        if (GraphStator.isInSameSector(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees(), target)) {
            addCommands(
                new RetimeArm(arm, target), // Make them arrive at the same time (not entirely needed)
                new ParallelToPos(arm, target, true), // Execute move
                new InstantCommand(() -> arm.resetMotionMagic()) // Reset our speed adjustments
            );

        // If there is only one intermediary waypoint, then only schedule one
        } else if (waypoints.length == 1) {
            addCommands(
                new RetimeArm(arm, waypoints[0]),
                new ParallelToPos(arm, waypoints[0], false),
                new RetimeArm(arm, target),
                new ParallelToPos(arm, target, true),
                new InstantCommand(() -> arm.resetMotionMagic())
            );

        // If there is not one, that means that there has to be two, so we schedule appropriately
        } else {
            addCommands(
                new RetimeArm(arm, waypoints[0]), // Make sure the bicep and wrist arrive at the same time
                new ParallelToPos(arm, waypoints[0], false), // Execute the move, then repeat
                new RetimeArm(arm, waypoints[1]),
                new ParallelToPos(arm, waypoints[1], false),
                new RetimeArm(arm, target),
                new ParallelToPos(arm, target, true), // We tell the arm that its the last move so that we can be more precise
                new InstantCommand(() -> arm.resetMotionMagic()) // Reset all of our speed adjustments
            );
        }

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }
}