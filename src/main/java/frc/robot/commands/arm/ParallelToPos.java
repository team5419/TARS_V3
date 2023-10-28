package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.arm.ArmState.Waypoint;

/**
 * @author 
 */
public class ParallelToPos extends ParallelCommandGroup {
    /**
     * This should only be used if you know what you are doing
     * @param arm
     * @param target
     */
    public ParallelToPos(OptimizedArm arm, Waypoint target) {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(arm);

        addCommands(
            new BicepToPos(arm, arm.degreesToTicksBicep(target.bicep)),
            new WristToPos(arm, arm.degreesToTicksWrist(target.wrist))
        );
    }

     /**
     * This should only be used if you know what you are doing
     * @param arm
     * @param target
     */
    public ParallelToPos(OptimizedArm arm, ArmTargets target) {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(arm);

        addCommands(
            new BicepToPos(arm, target.bicepTarget),
            new WristToPos(arm, target.wristTarget)
        );
    }
}