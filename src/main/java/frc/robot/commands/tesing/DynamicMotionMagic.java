package frc.robot.commands.tesing;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.OptimizedArm;

/**
 * The idea is to test if you can have a dynamic motion magic point. Don't know and currently the wifi is down
 * @author Grayson
 */
public class DynamicMotionMagic extends CommandBase {

    private final OptimizedArm arm;

    private double min = -40;
    private double max = 40;
    private double changePerTick = 0.5;

    private double currentDegrees = 0;
    private boolean isGoingUp = true;

    public DynamicMotionMagic(OptimizedArm arm) {
        this.arm = arm;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // arm.configMotionMagic(true, 2000, 8000, 2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (currentDegrees >= max) {
            isGoingUp = false;
        } else if (currentDegrees <= min) {
            isGoingUp = true;
        }

        if (isGoingUp) {
            currentDegrees += changePerTick;
        } else {
            currentDegrees -= changePerTick;
        }

        arm.setBicep(ControlMode.MotionMagic, arm.degreesToTicksBicep(currentDegrees));
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.resetMotionMagic();
        arm.setBicep(ControlMode.MotionMagic, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}