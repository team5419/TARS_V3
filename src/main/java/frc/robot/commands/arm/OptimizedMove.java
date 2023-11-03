package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;

/**
 * @author Grayson
 */
public class OptimizedMove extends SequentialCommandGroup {
    public OptimizedMove(OptimizedArm arm, ArmTargets target, ArmWaypoints[] waypoints) {

        arm.stop(); // Stop the arm
        arm.resetMotionMagic(); // Reset the motion magic

        if(waypoints[0] == null) { // This is only true if moving into invalid space
            System.err.println("[OPTIMIZED MOVE] Invalid position requested, abandoning move");
            return;
        }
        
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

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }
}