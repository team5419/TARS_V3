package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class TimeDrive extends CommandBase {

    private final Swerve swerve;
    private Timer timer;
    private double endTime;
    public TimeDrive(Swerve swerve, double time) {
        this.swerve = swerve;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
        timer = new Timer();
        this.endTime = time;
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerve.drive(new Translation2d(1, 0), 0, true,false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > endTime;
    }

}