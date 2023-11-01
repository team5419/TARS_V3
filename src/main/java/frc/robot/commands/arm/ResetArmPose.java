package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.OptimizedArm;

public class ResetArmPose extends SequentialCommandGroup {
    public ResetArmPose (OptimizedArm arm) {
        addCommands(
            new InstantCommand(() -> arm.resetArmPose())
        );
    }
}
