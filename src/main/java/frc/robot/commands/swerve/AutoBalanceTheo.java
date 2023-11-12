package frc.robot.commands.swerve;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 * @author 
 */
public class AutoBalanceTheo extends CommandBase {

    private final Swerve swerve;

    private enum State {
        GENERAL,
        CORRECTION
    }

    private double rollAngle;
	private double previousAngle;
    private State currentState;

    public AutoBalanceTheo(Swerve swerve) {
        this.swerve = swerve;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentState = State.GENERAL;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        rollAngle = swerve.gyro.getRoll();

        if(Math.abs(rollAngle - previousAngle) < 0.3 && Math.abs(rollAngle) < 11) {
            // currentState = currentState.
        }

        if(currentState == State.GENERAL) {

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    public boolean getFallingEdge() {
        return false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}