package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.OptimizedArm;

public class BicepToPosAuto extends CommandBase {

    private final OptimizedArm arm;
    private double target;

    public BicepToPosAuto(OptimizedArm arm, double target) {
        this.arm = arm;
        this.target = target;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setBicep(ControlMode.MotionMagic, target);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(target - arm.getBicepPosition()) < 1000;
    }

}