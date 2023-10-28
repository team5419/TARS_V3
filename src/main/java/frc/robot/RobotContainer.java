package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.commands.arm.TwoStageHighAuto;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.auto.TwoStageHighChoiced;
import frc.robot.commands.arm.MoveToPos;
import frc.robot.commands.swerve.LLAutoPickup;
import frc.robot.commands.swerve.SnapTo;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.tesing.DynamicMotionMagic;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.OptimizedArm;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    // private final Joystick driver = new Joystick(0);
    public final static CommandXboxController driver = new CommandXboxController(0);
    public final static CommandXboxController coDriver = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final OptimizedArm m_arm = new OptimizedArm(true);
    public final Intake mIntake = new Intake();
    private final Vision2 vision2 = new Vision2();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> driver.leftBumper().getAsBoolean()
            )
        );

        // Set up our events for our autos
        setUpEventMap();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * 
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //** Driver Buttons **//
        // Zero gyro
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        // Stow
        driver.a().onTrue(new MoveToPos(m_arm, stow));

        // Lock Swerve
        driver.x().onTrue(Commands.runOnce(() -> s_Swerve.lock()));

        // Auto Pickup?
        driver.back().whileTrue(new LLAutoPickup(s_Swerve, vision2));

        // Snap to shoot
        driver.rightBumper().whileTrue(new SnapTo(s_Swerve, 
            Rotation2d.fromDegrees(0), 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis),
            () -> driver.leftBumper().getAsBoolean()
        ));
        
            // Snap to substation
        driver.b().whileTrue(new SnapTo(s_Swerve, 
            Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? -90 : 90), 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis),
            () -> driver.leftBumper().getAsBoolean()
        ));

        // Run Rollers in
        driver.leftTrigger().whileTrue(Commands.runEnd(() -> mIntake.set(Constants.IntakeConstants.INTAKE_PCT), () -> mIntake.set(-0.2), mIntake));

        // Run rollers out
        driver.rightTrigger().whileTrue(Commands.runEnd(() -> mIntake.setVolts(Constants.IntakeConstants.OUTTAKE_VOLTS), () -> mIntake.set(0), mIntake));


        //** CO DRIVER BINDINGS **//
        // Stow
        coDriver.a().onTrue(new MoveToPos(m_arm, stow));

        // High
        coDriver.y().onTrue(new MoveToPos(m_arm, coneHigh));
        coDriver.povUp().onTrue(new MoveToPos(m_arm, cubeHigh));
        // coDriver.y().onTrue(new TwoPartHigh(m_arm, coneHigh)); //! Currently in beta
        // coDriver.povUp().onTrue(new TwoPartHigh(m_arm, cubeHigh)); //! Currently in beta

        // Mid
        coDriver.x().onTrue(new MoveToPos(m_arm, coneMid));
        coDriver.povRight().onTrue(new MoveToPos(m_arm, cubeMid));
        coDriver.povLeft().onTrue(new MoveToPos(m_arm, cubeMid));

        // Hybrid
        coDriver.povDown().onTrue(new MoveToPos(m_arm, cubeHybrid));
        coDriver.b().onTrue(new MoveToPos(m_arm, coneHybrid));

        // Ground Intakes
        coDriver.leftTrigger().onTrue(new MoveToPos(m_arm, cubeGround));
        coDriver.rightTrigger().onTrue(new MoveToPos(m_arm, coneGround));

        // Substations
        coDriver.leftBumper().onTrue(new MoveToPos(m_arm, cubeSubstation));
        coDriver.rightBumper().onTrue(new MoveToPos(m_arm, coneSubstation));

        driver.povRight().whileTrue(Commands.runEnd(
            () -> mIntake.set(-0.2 * INTAKE_PCT), 
            () -> mIntake.set(0)
        ));

        driver.povLeft().whileTrue(Commands.runEnd(
            () -> mIntake.set(INTAKE_PCT), 

            () -> mIntake.set(-0.075)
        ));

        driver.back().whileTrue(new DynamicMotionMagic(m_arm));
    
    }

    private void setUpEventMap() {
        HashMap<String, Command> map = Constants.AutoConstants.eventMap;

        map.put("ArmCubeGround", new MoveToPos(m_arm, cubeGround));
        map.put("ArmStow", new MoveToPos(m_arm, stow));
        map.put("ConstantIntakeStart", Commands.runOnce(() -> mIntake.set(-0.2)));
        map.put("RunIntake", Commands.runOnce(() -> mIntake.set(INTAKE_PCT)));

        map.put("TwoStageHighCube", new TwoStageHighChoiced(m_arm, mIntake, true));
        map.put("TwoStageHighCone", new TwoStageHighChoiced(m_arm, mIntake, false));

        map.put("ShootHybridCube", new ShootAuto(true, cubeHybrid, mIntake, m_arm));
        map.put("ShootMidCube", new ShootAuto(true, cubeMid, mIntake, m_arm));
        map.put("ShootHighCube", new ShootAuto(true, cubeHigh, mIntake, m_arm));

        map.put("ShootHybridCone", new ShootAuto(false, coneHybrid, mIntake, m_arm));
        map.put("ShootMidCone", new ShootAuto(false, coneMid, mIntake, m_arm));
        map.put("ShootHighCone", new ShootAuto(false, coneHigh, mIntake, m_arm));
    }
}
