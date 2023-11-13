package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.OptimizedArm;

/**
 * @author
 */
public class Stop extends CommandBase {

  private final OptimizedArm arm;

  public Stop(OptimizedArm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
