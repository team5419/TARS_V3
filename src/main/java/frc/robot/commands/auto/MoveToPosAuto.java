package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.commands.arm.WristToPos;
import frc.robot.subsystems.OptimizedArm;

public class MoveToPosAuto extends SequentialCommandGroup {
    public MoveToPosAuto(OptimizedArm arm, double bicepTarget, double wristTarget) {
        addCommands(
            new WristToPos(arm, 0),
            new BicepToPosAuto(arm, bicepTarget),
            new WristToPos(arm, wristTarget)
        );
        
    }

    public MoveToPosAuto(OptimizedArm arm, ArmTargets target) {
        addCommands(
            new WristToPos(arm, 0),
            new BicepToPosAuto(arm, target.bicepTarget),
            new WristToPos(arm, target.wristTarget)
        );
        
    }
}