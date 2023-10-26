package frc.robot.commands.swerve;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    BooleanSupplier slowMode;

    public SnapTo(Swerve swerve, Rotation2d target, DoubleSupplier translation, DoubleSupplier straif, BooleanSupplier slowMode) {
        this.swerve = swerve;
        this.target = target;
        this.translation = translation;
        this.straif = straif;
        this.slowMode = slowMode;

        controller = new PIDController(0.7, 0, 0);
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

        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(straif.getAsDouble(), Constants.stickDeadband);
        double rotationVal = controller.calculate(swerve.getPose().getRotation().getRadians(), target.getRadians());

        /* Slow down the robot to allow greater precision */
        if (slowMode.getAsBoolean()) {
            translationVal *= Constants.Swerve.slowModeSpeedMultiplier;
            strafeVal *= Constants.Swerve.slowModeSpeedMultiplier;
            rotationVal *= Constants.Swerve.slowModeTurnMultiplier;
        }

        /* Drive */
        swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );

        // swerve.drive(new Translation2d(translation.getAsDouble(), straif.getAsDouble()), rotationVal, true, false);
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