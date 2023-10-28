package frc.robot.commands.arm;


import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmState.SectorState;
import frc.robot.subsystems.arm.ArmState.Waypoint;
import frc.robot.subsystems.arm.OptimizedArm;

/**
 * @author Grayson
 */
public class OptimizedMove extends SequentialCommandGroup {
    public OptimizedMove(OptimizedArm arm, ArmTargets target) {
        ArmState graphStator = arm.getGraphStator();

        // If we are in the same sector, then we are good to move wherever
        if (graphStator.isInSameSector(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees(), target)) {
            addCommands(new ParallelToPos(arm, target));
            return;
        }

        // "trace" our path trough our imaginary graph to find all of our waypoints
        Waypoint[] waypoints = graphStator.tracePath(
            graphStator.newWaypoint(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()), // Starting
            graphStator.newWaypoint(arm.ticksToDegreesBicep(target.bicepTarget), arm.ticksToDegreesWrist(target.wristTarget)) // Ending
        );

        for (Waypoint waypoint : waypoints) {
            // Really hoping that you can call addCommands more than once
            if(waypoint != null) addCommands(new ParallelToPos(arm, waypoint));
        }

        // ! a lot of degrees/encoder conversion problems (maybe?)

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }
}