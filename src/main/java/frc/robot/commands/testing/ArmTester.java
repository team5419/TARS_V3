package frc.robot.commands.testing;

import static java.lang.Math.abs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.arm.Waypoint;

/**
 * @author 
 */
public class ArmTester extends CommandBase {

    private final OptimizedArm arm;
    private ArmTargets target;
    private GenericEntry bicep, wrist;

    public ArmTester(OptimizedArm arm, GenericEntry bicep, GenericEntry wrist) {
        this.arm = arm;
        this.target = target;

        this.bicep = bicep;
        this.wrist = wrist;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        System.out.println("Target: " + bicep.getDouble(0) + " - " + wrist.getDouble(0));

        ArmWaypoints[] waypoints = GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
            new ArmTargets().fromDegrees(bicep.getDouble(0), wrist.getDouble(0))
        );

        if(waypoints.length == 0) {
            System.out.println("In same sector");
            // GraphStator.calculateNewMotionMagic(new Waypoint(0, 0), new Waypoint(bicep.getDouble(0), wrist.getDouble(0)), arm);
            return;
        }

        if(waypoints[0] == null) {
            System.err.println("Invalid position requested, aborting");
            return;
        }

        System.out.println(waypoints.length);

        for (ArmWaypoints waypoint : waypoints) {
            System.out.println(waypoint.point.bicep +  " " + waypoint.point.wrist + " INTERMEDIATE POINT");
            // GraphStator.calculateNewMotionMagic(new Waypoint(0, 0), new Waypoint(waypoint.point.bicep, waypoint.point.wrist), arm);
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}