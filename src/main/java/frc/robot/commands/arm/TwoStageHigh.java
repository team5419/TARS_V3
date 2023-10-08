package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class TwoStageHigh extends SequentialCommandGroup {
    public TwoStageHigh(Arm arm, Intake intake) {
        addCommands(
            new InstantCommand(() -> intake.set(Constants.IntakeConstants.INTAKE_PCT)),
            new MoveToPos (arm, 0, 0), // Make sure we are safe
            new BicepToPos(arm, Constants.ArmConstants.coneHigh.bicepTarget - arm.degreesToTicksBicep(15)), // move up + past to make sure wrist doesn't hit
            new WristToPos(arm, Constants.ArmConstants.coneHigh.wristTarget), // Move wrist
            new WaitCommand(0.5),
            new BicepToPos(arm, Constants.ArmConstants.coneHigh.bicepTarget), // Finalize arm
            new WaitCommand(1),
            new InstantCommand(() -> intake.setVolts(Constants.IntakeConstants.OUTTAKE_VOLTS)),
            new WaitCommand(1),
            new InstantCommand(() -> intake.set(0)),
            new BicepToPos(arm, Constants.ArmConstants.coneHigh.bicepTarget - arm.degreesToTicksBicep(10)), // move up + past to make sure wrist doesn't hit
            new MoveToPos(arm, 0, 0)
        );        
    }
}