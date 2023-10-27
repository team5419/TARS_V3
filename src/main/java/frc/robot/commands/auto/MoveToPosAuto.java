package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.commands.arm.WristToPos;
import frc.robot.subsystems.Arm;

public class MoveToPosAuto extends SequentialCommandGroup {
    public MoveToPosAuto(Arm arm, double bicepTarget, double wristTarget) {
        addCommands(
            new WristToPos(arm, 0),
            new BicepToPosAuto(arm, bicepTarget),
            new WristToPos(arm, wristTarget)
        );
        
    }

    public MoveToPosAuto(Arm arm, ArmTargets target) {
        addCommands(
            new WristToPos(arm, 0),
            new BicepToPosAuto(arm, target.bicepTarget),
            new WristToPos(arm, target.wristTarget)
        );
        
    }
}