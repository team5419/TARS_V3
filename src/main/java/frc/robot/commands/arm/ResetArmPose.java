package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.OptimizedArm;

public class ResetArmPose extends CommandBase {

  private OptimizedArm arm;

  public ResetArmPose(OptimizedArm arm) {
    this.arm = arm;
  }

  public void initialize() {
    arm.resetArmPose();

    if (arm.getCurrentCommand() != null) {
      arm.getCurrentCommand().cancel();
    }
  }

  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
