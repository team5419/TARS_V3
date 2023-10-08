package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    public final Arm m_arm = new Arm(false);
    private final Intake mIntake = new Intake();

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

        // mIntake.setDefaultCommand(
        //     new ConstantIntake(mIntake)
        // );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // Stow
        driver.a().onTrue(new MoveToPos(m_arm, stow));
        // Run Rollers in
        driver.leftTrigger().whileTrue(Commands.runEnd(() -> mIntake.set(Constants.IntakeConstants.INTAKE_PCT), () -> mIntake.set(-0.1), mIntake));
        // Run rollers out
        driver.rightTrigger().whileTrue(Commands.runEnd(() -> mIntake.setVolts(Constants.IntakeConstants.OUTTAKE_VOLTS), () -> mIntake.set(0), mIntake));
        
        // Stow
        coDriver.a().onTrue(new MoveToPos(m_arm, stow));
        // High
        coDriver.y().onTrue(new MoveToPos(m_arm, high));
        // Mid
        coDriver.x().onTrue(new MoveToPos(m_arm, mid));
        // Ground Intake
        coDriver.leftBumper().onTrue(new MoveToPos(m_arm, ground));
        // Substation
        coDriver.rightBumper().onTrue(new MoveToPos(m_arm, substation));

        coDriver.b().onTrue(
            new SequentialCommandGroup(
                new MoveToPos(m_arm, -2000, -2000),
                new MoveToPos(m_arm, 2000, 2000)
            )
        );


        
        // coDriver.x().onTrue(
        //     new SequentialCommandGroup(
        //         new MoveToPos(m_arm, -72132, -32472),
        //         new PrintCommand("ONTO SECOND MOVE"),
        //         new MoveToPos(m_arm, -63511, -42830)
        // ));


        // driver.b().onTrue(new TwoPartHigh(m_arm));
    
        // Mid?
//         driver.x().onTrue(m_arm.runOnce(() -> {
//             m_arm.setArmMid();
//             mIntake.set(INTAKE_PCT);
//         }));

//         driver.a().onTrue(m_arm.runOnce(() -> {
//             m_arm.setArmStow();
//             System.out.println("ARM SHOULD BE STOWED");
//             // m_arm.passSetpoints(0, 200);
//         }));
    
    
//         driver.leftBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(LOW_BASE_POS_CUBE, LOW_WRIST_POS_CUBE);}));
//             //driver.leftBumper().whileTrue(new ChassisDriveToNearestTarget(m_chassis, m_cameras,99));
//             //driver.leftBumper().onFalse(m_chassis.runOnce(() -> {m_chassis.setPrecisionTrue();}));
//     //    driver.leftBumper().whileTrue(m_chassis.run(() -> {m_chassis.crossWheels();}));
//     //    driver.leftBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 30*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
//             // driver.rightBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 30*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
//         driver.rightBumper().whileTrue(mIntake.run(() -> {
//             mIntake.set(INTAKE_PCT);
//         }));
//         driver.rightBumper().onTrue(m_arm.runOnce(() -> {
//             m_arm.setArmConeIntake();
//             mIntake.set(INTAKE_PCT);
//         }));
//         driver.rightBumper().onFalse(mIntake.runOnce(() -> {
//             mIntake.set(-0.075);
//         }));
    
//     //    driver.rightBumper().whileTrue(new ChassisTargetToCone(m_chassis, m_cameras));
    
//         driver.povUp().onTrue(m_arm.runOnce(() -> {
//             m_arm.setTalonTargets(m_arm.baseTalonTarget - 1000, m_arm.wristTalonTarget);
//         }));
//         driver.povDown().onTrue(m_arm.runOnce(() -> {
//             m_arm.setTalonTargets(m_arm.baseTalonTarget + 1000, m_arm.wristTalonTarget);
//         }));

//         driver.povRight().whileTrue(mIntake.run(() -> {
//             mIntake.set(-0.2 * INTAKE_PCT);
//         }));
//         driver.povRight().onFalse(mIntake.runOnce(() -> {
//             mIntake.set(0.0);
//         }));
//         driver.povLeft().whileTrue(mIntake.run(() -> {
//             mIntake.set(INTAKE_PCT);
//         }));
//         driver.povLeft().onFalse(mIntake.runOnce(() -> {
//             mIntake.set(-0.075);
//         }));
    
//         driver.rightTrigger().onTrue(m_arm.runOnce(() -> {
//             m_arm.setArmCubeIntake();
//         }));
//         // driver.rightTrigger(0.7).whileTrue(new IntakeCube(mIntake, INTAKE_PCT));
//         driver.rightTrigger(0.7).onFalse(mIntake.runOnce(() -> {
//             mIntake.set(-0.2);
//         }));

    
//             // driver.leftTrigger().onTrue(m_chassis.runOnce(() -> {m_chassis.setPrecisionFalse();}));
//         driver.leftTrigger(0.7).whileTrue(mIntake.run(() -> {
//             if(m_arm.hasCone) {
//                 mIntake.setVolts(OUTTAKE_VOLTS);
//             }else{
//                 mIntake.setVolts(OUTTAKE_VOLTS_CUBE);
//             }
//         }));
//         driver.leftTrigger(0.7).onFalse(mIntake.runOnce(() -> {
//             mIntake.set(-0.0);
//         }));
// //        driver.leftTrigger(0.7).onFalse(m_chassis.runOnce(() -> {
//     //            m_chassis.setPrecisionFalse();
//     //        }));
    
//             driver.back().onTrue(m_arm.runOnce(() -> {
//                 m_arm.setTalonTargets(CHUTE_BASE_POS, CHUTE_WRIST_POS);
//             }));
//             driver.back().onTrue(m_arm.runOnce(() -> {
//                 m_arm.hasCone = true;
//             }));
            
//             // driver.start().onTrue(new CubeFling(mIntake, m_arm));
    
//     //        driver.y().and(driver.rightBumper()).onTrue(m_arm.runOnce(() -> {
//     //            m_arm.setTalonTargets(SHELF_BASE_POS, SHELF_WRIST_POS);
//     //        }));
    
//             // driver.start().whileTrue(new ChassisDriveToNearestTarget(m_chassis, m_cameras, 99.0));
    
//             //driver.start().whileTrue(new ChassisDriveToNearestTarget(m_chassis, m_cameras, 99.0));
//             // driver.start().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(LOW_BASE_POS_CUBE, LOW_WRIST_POS_CUBE);}));
//     //        driver.start().onTrue(m_chassis.runOnce(() -> {m_chassis.resetCustomOdoToOrigin();}));
        }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
