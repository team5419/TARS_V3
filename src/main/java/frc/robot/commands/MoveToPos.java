package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.Arm;

public class MoveToPos extends SequentialCommandGroup {
    public MoveToPos(Arm arm, double bicepTarget, double wristTarget) {
        addRequirements(arm);
        addCommands(
            new WristToPos(arm, 0),
            new BicepToPos(arm, bicepTarget),
            new WristToPos(arm, wristTarget)
        );
        
    }

    public MoveToPos(Arm arm, ArmTargets target) {
        addRequirements(arm);
        addCommands(
            new WristToPos(arm, 0),
            new BicepToPos(arm, target.bicepTarget),
            new WristToPos(arm, target.wristTarget)
        );
        
    }
}