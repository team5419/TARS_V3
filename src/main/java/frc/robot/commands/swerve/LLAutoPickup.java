package frc.robot.commands.swerve;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2;
import frc.robot.subsystems.limelight.LimelightHelpers;;


/**
 * @author Grayson 
 */
public class LLAutoPickup extends CommandBase {

    private final Swerve swerve;
    private final Vision2 vision;

    private final PIDController translationController;
    private final PIDController straifController;
    private final PIDController rotationController;

    public LLAutoPickup(Swerve swerve, Vision2 vision) {
        this.swerve = swerve;
        this.vision = vision;

        translationController = new PIDController(0.1, 0, 0);
        straifController = new PIDController(0.1, 0, 0);
        rotationController = new PIDController(0.1, 0, 0);

        translationController.setSetpoint(0);
        straifController.setSetpoint(0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        translationController.reset();
        straifController.reset();
        rotationController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");

        double newTranslation = translationController.calculate(ty);
        double newStraif = translationController.calculate(tx);
        double newRotation = rotationController.calculate(swerve.getPose().getRotation().getDegrees(), 0);

        swerve.drive(new Translation2d(newTranslation, newStraif), newRotation, false, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return vision.isLimelightAlive == false ||  MathUtil.applyDeadband(LimelightHelpers.getTY(""), 0.01) == 0 && MathUtil.applyDeadband(LimelightHelpers.getTX(""), 0.01) == 0;
    }

}