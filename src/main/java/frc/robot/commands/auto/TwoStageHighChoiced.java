package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.MoveToPos;
import frc.robot.commands.arm.WristToPos;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.OptimizedArm;

public class TwoStageHighChoiced extends SequentialCommandGroup {
    public TwoStageHighChoiced(OptimizedArm arm, Intake intake, boolean isCube) {
        addCommands(
            new InstantCommand(() -> intake.set(Constants.IntakeConstants.INTAKE_PCT)),
            new MoveToPos (arm, 0, 0), // Make sure we are safe
            new BicepToPosAuto(arm, Constants.ArmConstants.cubeHigh.bicepTarget + arm.degreesToTicksBicep(15)),
            new WristToPos(arm, Constants.ArmConstants.cubeHigh.wristTarget),
            new WaitCommand(1),
            new InstantCommand(() -> intake.setVolts(isCube ? Constants.IntakeConstants.OUTTAKE_VOLTS_CUBE : Constants.IntakeConstants.OUTTAKE_VOLTS)),
            new WaitCommand(0.2),
            new BicepToPosAuto(arm, Constants.ArmConstants.cubeHigh.bicepTarget + arm.degreesToTicksBicep(15)),
            new WristToPos(arm, 0),
            new BicepToPosAuto(arm, 0)
        );
    }
}
