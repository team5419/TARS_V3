package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmTargets;
import frc.robot.commands.arm.MoveToPos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ShootAuto extends SequentialCommandGroup {
    public ShootAuto (boolean isCube, ArmTargets target, Intake intake, Arm arm) {
        addCommands(
            Commands.runOnce(() -> intake.set(Constants.IntakeConstants.INTAKE_PCT)),
            new MoveToPosAuto(arm, target),
            new WaitCommand(0.2),
            Commands.runOnce(() -> intake.setVolts(isCube ? Constants.IntakeConstants.OUTTAKE_VOLTS_CUBE : Constants.IntakeConstants.OUTTAKE_VOLTS)), 
            new WaitCommand(0.5),
            Commands.runOnce(() -> intake.set(0))
        );
    }
}
