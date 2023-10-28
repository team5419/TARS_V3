package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.OptimizedArm;

public class MoveToPos extends SequentialCommandGroup {
    public MoveToPos(OptimizedArm arm, double bicepTarget, double wristTarget) {
        addRequirements(arm);
        addCommands(
            new WristToPos(arm, 0),
            new BicepToPos(arm, bicepTarget),
            new WristToPos(arm, wristTarget)
        );
        
    }

    public MoveToPos(OptimizedArm arm, ArmTargets target) {
        addRequirements(arm);
        addCommands(
            new WristToPos(arm, 0),
            new BicepToPos(arm, target.bicepTarget),
            new WristToPos(arm, target.wristTarget)
        );
        
    }
}