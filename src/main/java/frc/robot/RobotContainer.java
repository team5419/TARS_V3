package frc.robot;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.ArmThrow;
import frc.robot.commands.arm.MoveToPos;
import frc.robot.commands.arm.OnTheFlyCommand;
import frc.robot.commands.arm.OptimizedMove;
import frc.robot.commands.arm.OptimizedMoveV3;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.lights.AnimateLights;
import frc.robot.commands.swerve.AutoAlign;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SnapTo;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.OptimizedArm;
import java.util.HashMap;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  // private final Joystick driver = new Joystick(0);
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController coDriver = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final OptimizedArm m_arm = new OptimizedArm(false);
  public final Intake mIntake = new Intake();
  public final Lights leds = new Lights(52);
  private final Vision2 vision2 = new Vision2(s_Swerve);

  public GenericEntry bicepTuningEntry =
      Shuffleboard.getTab("Tuning")
          .add("Bicep dest", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -150, "max", 150))
          .getEntry();

  public GenericEntry wristTuningEntry =
      Shuffleboard.getTab("Tuning")
          .add("Wrist dest", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", -215, "max", 215))
          .getEntry();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> driver.leftBumper().getAsBoolean()));

    // Make our leds led
    leds.setDefaultCommand(new AnimateLights(leds, s_Swerve, mIntake));

    // Set up our events for our autos
    setUpEventMap();

    // Configure the button bindings
    configureButtonBindings();

    // Make driver station stop repeating that we are missing a controller
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   *
   * <p>instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ** Driver Buttons **//
    // Zero gyro
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    // Stow
    driver.a().onTrue(new MoveToPos(m_arm, stow));

    // Lock Swerve
    driver.x().onTrue(Commands.runOnce(() -> s_Swerve.lock()));

    // Auto align
    // driver.back().whileTrue(new AutoAlign(s_Swerve, vision2, 0.15));

    // Auto Balance
    driver.povUp().whileTrue(new AutoBalance(s_Swerve));

    // Testing
    // driver.povDown().whileTrue(new ArmTester(m_arm, bicepTuningEntry, wristTuningEntry));
    // driver.povLeft().onTrue(new ParallelToPos(m_arm, coneGround, false));

    driver
        .povRight()
        .whileTrue(Commands.runEnd(() -> mIntake.set(-0.2 * INTAKE_PCT), () -> mIntake.set(0)));

    driver
        .povLeft()
        .whileTrue(Commands.runEnd(() -> mIntake.set(INTAKE_PCT), () -> mIntake.set(-0.075)));

    // Snap to shoot
    driver
        .rightBumper()
        .whileTrue(
            new SnapTo(
                s_Swerve,
                Rotation2d.fromDegrees(0),
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> driver.leftBumper().getAsBoolean()));

    // Snap to substation
    driver
        .b()
        .whileTrue(
            new SnapTo(
                s_Swerve,
                Rotation2d.fromDegrees(DriverStation.getAlliance() == Alliance.Red ? 90 : -90),
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> driver.leftBumper().getAsBoolean()));

    // Run Rollers in
    driver
        .leftTrigger()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  mIntake.set(Constants.IntakeConstants.INTAKE_PCT);
                  s_Swerve.isIntakeActive = true;
                },
                () -> {
                  mIntake.set(-0.2);
                  s_Swerve.isIntakeActive = false;
                },
                mIntake));

    // Run rollers out
    driver
        .rightTrigger()
        .whileTrue(
            Commands.runEnd(
                () -> mIntake.setVolts(Constants.IntakeConstants.OUTTAKE_VOLTS),
                () -> mIntake.set(0),
                mIntake));

    // ** CO DRIVER BINDINGS **//

    // Stow
    coDriver.a().onTrue(new OptimizedMoveV3(m_arm, stow));

    // High
    coDriver.y().onTrue(new OptimizedMoveV3(m_arm, coneHigh));
    coDriver.povUp().onTrue(new OptimizedMoveV3(m_arm, cubeHigh));

    // Mid
    coDriver.x().onTrue(new OptimizedMoveV3(m_arm, coneMid));
    coDriver.povRight().onTrue(new MoveToPos(m_arm, cubeMid));
    coDriver.povLeft().onTrue(new OptimizedMoveV3(m_arm, cubeMid));

    // Hybrid
    coDriver.povDown().onTrue(new OptimizedMoveV3(m_arm, cubeHybrid));
    coDriver.b().onTrue(new OptimizedMoveV3(m_arm, coneHybrid));

    // Ground Intakes
    coDriver
        .leftTrigger()
        .onTrue(new OptimizedMoveV3(m_arm, cubeGround))
        .onTrue(Commands.runOnce(() -> s_Swerve.isUsingCones = false));
    coDriver
        .rightTrigger()
        .onTrue(new OptimizedMoveV3(m_arm, coneGround))
        .onTrue(Commands.runOnce(() -> s_Swerve.isUsingCones = true));

    // Substations
    coDriver
        .leftBumper()
        .onTrue(new OptimizedMoveV3(m_arm, cubeSubstation))
        .onTrue(Commands.runOnce(() -> s_Swerve.isUsingCones = false));
    coDriver
        .rightBumper()
        .onTrue(new OptimizedMoveV3(m_arm, coneSubstation))
        .onTrue(Commands.runOnce(() -> s_Swerve.isUsingCones = true));

    // Change LEDS
    coDriver.back().onTrue(Commands.runOnce(() -> s_Swerve.isUsingCones = !s_Swerve.isUsingCones));

    // Fling
    coDriver
        .rightStick()
        .onTrue(new OnTheFlyCommand(() -> new ArmThrow(m_arm, mIntake, coneHigh), mIntake, m_arm));
  }

  private void setUpEventMap() {
    HashMap<String, Command> map = Constants.AutoConstants.eventMap;

    map.put("TimedAutoAlign", new AutoAlign(s_Swerve, vision2, 0.2, 1.1));

    map.put("Balance", new AutoBalance(s_Swerve));

    map.put("ArmCubeGround", new OnTheFlyCommand(() -> new OptimizedMove(m_arm, cubeGround)));
    map.put("ArmConeGround", new OnTheFlyCommand(() -> new OptimizedMove(m_arm, coneGround)));

    map.put("ArmStow", new MoveToPos(m_arm, stow));
    map.put("ConstantIntakeStart", Commands.runOnce(() -> mIntake.set(-0.4)));
    map.put("RunIntake", Commands.runOnce(() -> mIntake.set(INTAKE_PCT)));

    map.put(
        "ShootCone",
        new SequentialCommandGroup(
            new InstantCommand(() -> mIntake.set(OUTTAKE_VOLTS)),
            new WaitCommand(0.2),
            new InstantCommand(() -> mIntake.set(0))));

    map.put(
        "ShootCube",
        new SequentialCommandGroup(
            new InstantCommand(() -> mIntake.set(OUTTAKE_VOLTS)),
            new WaitCommand(0.2),
            new InstantCommand(() -> mIntake.set(0))));

    // map.put("TwoStageHighCube", new TwoStageHighChoiced(m_arm, mIntake, true));
    // map.put("TwoStageHighCone", new TwoStageHighChoiced(m_arm, mIntake, false));

    map.put("ShootHybridCube", new ShootAuto(true, cubeHybrid, mIntake, m_arm));
    map.put("ShootMidCube", new ShootAuto(true, cubeMid, mIntake, m_arm));
    map.put("ShootHighCube", new ShootAuto(true, cubeHigh, mIntake, m_arm));

    map.put("ShootHybridCone", new ShootAuto(false, coneHybrid, mIntake, m_arm));
    map.put("ShootMidCone", new ShootAuto(false, coneMid, mIntake, m_arm));
    map.put("ShootHighCone", new ShootAuto(false, coneHigh, mIntake, m_arm));

    map.put("ArmToHybridCone", new OnTheFlyCommand(() -> new OptimizedMove(m_arm, coneHybrid), m_arm));
    map.put("ArmToMidCone", new OnTheFlyCommand(() -> new OptimizedMove(m_arm, coneMid), m_arm));
    map.put("ArmToHighCone", new OnTheFlyCommand(() -> new OptimizedMove(m_arm, coneHigh), m_arm));

    map.put("ArmToHybridCube", new OnTheFlyCommand(() -> new OptimizedMove(m_arm, cubeHybrid), m_arm));
    map.put("ArmToMidCube", new OnTheFlyCommand(() -> new OptimizedMove(m_arm, cubeMid), m_arm));
    map.put("ArmToHighCube", new OnTheFlyCommand(() -> new OptimizedMove(m_arm, cubeHigh), m_arm));
  }
}
