package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;

/**
 * @author G (embarrassingly)
 */
public class OptimizedMoveWrapper extends CommandBase {

    private final OptimizedArm arm;
    private ArmTargets target;

    public OptimizedMoveWrapper(OptimizedArm arm, ArmTargets target) {
        this.arm = arm;        
        this.target = target;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmWaypoints[] waypoints = GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()),
            target
        );

        new OptimizedMove(arm, target, waypoints).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}