package frc.robot.commands.swerve;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 * @author Grayson
 */
public class SnapTo extends CommandBase {

    private final Swerve swerve;
    private Rotation2d target;
    private PIDController controller;

    DoubleSupplier translation;
    DoubleSupplier straif;

    public SnapTo(Swerve swerve, Rotation2d target, DoubleSupplier translation, DoubleSupplier straif) {
        this.swerve = swerve;
        this.target = target;
        this.translation = translation;
        this.straif = straif;

        controller = new PIDController(0.5, 0, 0);
        controller.enableContinuousInput(-Math.PI, Math.PI);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        controller.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double nextMove = controller.calculate(swerve.getPose().getRotation().getRadians(), target.getRadians());
        swerve.drive(new Translation2d(translation.getAsDouble(), straif.getAsDouble()), nextMove, true, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}