package frc.robot.commands.arm;

import static java.lang.Math.pow;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.arm.Waypoint;

/**
 * @author Grayson
 */
public class OptimizedMove extends SequentialCommandGroup {
    public OptimizedMove(OptimizedArm arm, ArmTargets target) {
        arm.stop(); // Stop the arm
        arm.resetMotionMagic(); // Reset the motion magic

        // "trace" our path trough our imaginary graph to find all of our waypoints
        ArmWaypoints[] waypoints = GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
            target
        );

        // if(waypoints.length == 0) { // If we are in the same sector
        //      addCommands(
        //         new RetimeArm(arm, target), // Make them arrive at the same time (not entirely needed)
        //         new ParallelToPos(arm, target, true), // Execute move
        //         new InstantCommand(() -> arm.resetMotionMagic()) // Reset our speed adjustments
        //     );
        //     return;
        // }
        
        for (ArmWaypoints point : waypoints) {
            addCommands(
                new PrintCommand("Currently in " + GraphStator.getSectorStateFromCoords(new ArmState(point.point.bicep, point.point.wrist))),
                new PrintCommand("GOING TO " + point.name()),
                new RetimeArm(arm, point),
                new ParallelToPos(arm, point, false)
            );
        }
        
        addCommands(
            new PrintCommand("Currently in " + GraphStator.getSectorStateFromCoords(new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()))),
            new PrintCommand("GOING TO " + GraphStator.getSectorStateFromCoords(new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees())).name()),
            new RetimeArm(arm, target), // Make them arrive at the same time (not entirely needed)
            new ParallelToPos(arm, target, true), // Execute move
            new InstantCommand(() -> arm.resetMotionMagic()) // Reset our speed adjustments
        );

        // if(waypoints[0] == null) { // This is only true if moving into invalid space
        //     System.err.println("[OPTIMIZED MOVE] Invalid position requested, abandoning move");
        //     return;
        // }

        // // If there is only one intermediary waypoint
        // if (waypoints.length == 1) {
        //     addCommands(
        //         new RetimeArm(arm, waypoints[0]),
        //         new ParallelToPos(arm, waypoints[0], false),
        //         new RetimeArm(arm, target),
        //         new ParallelToPos(arm, target, true),
        //         new InstantCommand(() -> arm.resetMotionMagic())
        //     );
        // // If there is not one, the only other return value can be two points, so we add the two points
        // } else {
        //     addCommands(
        //         new RetimeArm(arm, waypoints[0]), // Make sure the bicep and wrist arrive at the same time
        //         new ParallelToPos(arm, waypoints[0], false), // Execute the move, then repeat
        //         new RetimeArm(arm, waypoints[1]),
        //         new ParallelToPos(arm, waypoints[1], false),
        //         new RetimeArm(arm, target),
        //         new ParallelToPos(arm, target, true), // We tell the arm that its the last move so that we can be more precise
        //         new InstantCommand(() -> arm.resetMotionMagic()) // Reset all of our speed adjustments
        //     );
        // }

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }
}