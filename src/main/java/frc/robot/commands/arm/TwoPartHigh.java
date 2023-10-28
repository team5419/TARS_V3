package frc.robot.commands.arm;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.arm.OptimizedArm;

/**
 * @author Grayson
 */
public class TwoPartHigh extends SequentialCommandGroup {
    public TwoPartHigh(OptimizedArm arm, ArmTargets target) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);

        addCommands(
            new WristToPos(arm, 0), // Make sure were in a safe zone
            new BicepToPos(arm, target.bicepTarget - arm.degreesToTicksBicep(15)), // Move 15 degrees over our target (- bc its on the backside of the bot, and we want to move further)
            new WristToPos(arm, target.wristTarget), // Extend the wrist, now knowing that we are in the clear
            new BicepToPos(arm, target.bicepTarget) // Finally, lower the bicep to the correct position
        );
    }
}