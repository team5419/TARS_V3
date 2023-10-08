package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class WristToPos extends CommandBase {

    private final Arm arm;
    private double target;

    public WristToPos(Arm arm, double target) {
        this.arm = arm;
        this.target = target;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.wristTalon.set(ControlMode.MotionMagic, target);
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
        return Math.abs(target - arm.wristTalon.getSelectedSensorPosition()) < 1000;
    }

}