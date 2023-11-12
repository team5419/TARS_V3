package frc.robot.commands.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 * @author 
 */
public class AutoBalance extends CommandBase {

    private final Swerve swerve;
    private Pigeon2 gyro;

    LinearFilter filter = LinearFilter.highPass(2, 0.2);
    double calculatedRoll;
    double threshold = 1;
    double angleEpsilon;

    enum State {
        INITIAL,
        GENERAL,
        CORRECTION
    }

    State currentState;

    // GenericEntry rollLogger = Shuffleboard.getTab("Auto Balance").add("Roll", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
    // GenericEntry rollLoggerCalculated = Shuffleboard.getTab("Auto Balance").add("Filtered Roll", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
    // GenericEntry rollState = Shuffleboard.getTab("Auto Balance").add("State", State.GENERAL.toString()).withWidget(BuiltInWidgets.kTextView).getEntry();
    
    // GenericEntry thresholdEntry = Shuffleboard.getTab("Auto Balance").add("Threshold", 0.0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    // GenericEntry angleEpsilonEntry = Shuffleboard.getTab("Auto Balance").add("Angle Epsilon", 0.0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

    public AutoBalance(Swerve swerve) {
        this.swerve = swerve;
        gyro = swerve.gyro;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentState = State.INITIAL;

        filter.reset();

        // threshold = thresholdEntry.getDouble(10);
        threshold = 4.5;
        angleEpsilon = 1;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        calculatedRoll = filter.calculate(gyro.getRoll());
        double forward = 0;

        // rollLogger.setDouble(gyro.getRoll());
        // rollLoggerCalculated.setDouble(calculatedRoll);
        // rollState.setString(currentState.toString());

        // if(calculatedRoll < -threshold) {
        //     currentState = State.GENERAL;
        // } else if (calculatedRoll < threshold && currentState != State.INITIAL) {
        //     forward = -0.8;
        //     currentState = State.CORRECTION;
        // } else if (currentState != State.CORRECTION) {
        //     forward = 0.5;
        // }

        if (currentState == State.INITIAL) {
            forward = 1;

            if(calculatedRoll > threshold) {
                currentState = State.GENERAL;
            }
        } else if (currentState == State.GENERAL) {
            forward = 0.5;

            if(calculatedRoll < -threshold) {
                currentState = State.CORRECTION;
            }
        } else if (currentState == State.CORRECTION) {
            forward = -0.2;
        }

        swerve.drive(new Translation2d(forward, 0), 0, true, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.lock();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return false;
        return MathUtil.applyDeadband(gyro.getRoll(), angleEpsilon) == 0 && currentState == State.CORRECTION;
    }

}