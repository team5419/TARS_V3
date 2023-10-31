package frc.robot.commands.tesing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;

/**
 * @author 
 */
public class ArmTester extends CommandBase {

    private final OptimizedArm arm;
    private ArmTargets target;

    public ArmTester(OptimizedArm arm, ArmTargets target) {
        this.arm = arm;
        this.target = target;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmWaypoints[] waypoints = GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
            target
        );

        if(waypoints[0] == null) {
            System.err.println("Invalid position requested, continuing test");
        }

        System.out.println(waypoints.length);

        for (ArmWaypoints waypoint : waypoints) {
            System.out.println(waypoint);
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