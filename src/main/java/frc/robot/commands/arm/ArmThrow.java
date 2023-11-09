package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.OptimizedArm;

public class ArmThrow extends ParallelCommandGroup {
    public ArmThrow (OptimizedArm arm, Intake intake, ArmTargets target) {
        addCommands(
            new OptimizedMove(arm, target), // Move
            new SequentialCommandGroup(
                // new WaitCommand(0.2), // Delay the shot
                new WaitUntilCommand(() -> {
                    return arm.isAt(OptimizedArm.degreesToTicksBicep(80), OptimizedArm.degreesToTicksWrist(90));
                }),
                new InstantCommand(() -> intake.set(50)), // Shoot
                new WaitCommand(0.5), // Wait a sec
                new InstantCommand(() -> intake.set(0)) // Stop moving
            )
        );
    }
}
