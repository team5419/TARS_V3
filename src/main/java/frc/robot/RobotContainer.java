package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.*;
import frc.robot.commands.intake.IntakeCube;
import frc.robot.commands.intake.RunVoltsTime;
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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final Arm m_arm = new Arm();
    private final Intake mIntake = new Intake();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
            )
        );

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
            // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
            // new Trigger(m_exampleSubsystem::exampleCondition)
            //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    
            // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
            // // cancelling on release.
            // driver.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    //    driver.x().whileTrue(m_chassis.run(() -> {m_chassis.crossWheels();}));
    //    driver.a().onTrue(m_cameras.runOnce(() -> {m_cameras.togglePipeline();}));
    
    //         driver.y().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(HIGH_BASE_POS_VIKES, HIGH_WRIST_POS_VIKES);}));
    //        driver.y().onTrue(m_arm.runOnce(() -> {
    //            m_arm.setArmHigh();
    //        }));
    
             driver.y().onTrue(new TwoPartHigh(m_arm));
    
    //        driver.y().onTrue(new TwoPartHigh(m_arm));
            // driver.y().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(MID_BASE_POS, -190*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
    
            // driver.x().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(MID_BASE_POS, MID_WRIST_POS);}));
            driver.x().onTrue(m_arm.runOnce(() -> {
                m_arm.setArmMid();
                mIntake.set(INTAKE_PCT);
            }));
    //    driver.x().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(MID_BASE_POS, 0);}));
    //    driver.y().whileTrue(m_arm.run(() -> {m_arm.passSetpoints(PI/2/(PI/1024/BASE_GEAR_RATIO), 0);}));
    //    driver.y().onFalse(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 0);}));
    //    driver.y().whileTrue(new ManageArm(m_arm));
    
            driver.a().onTrue(m_arm.runOnce(() -> {
                m_arm.setArmStow();
            }));
    
    
            //driver.leftBumper().whileTrue(new ChassisAutoBalanceNew(m_chassis));
            // driver.leftBumper().whileTrue(new ChassisAutoBalanceFast(m_chassis));
            driver.leftBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(LOW_BASE_POS_CUBE, LOW_WRIST_POS_CUBE);}));
            //driver.leftBumper().whileTrue(new ChassisDriveToNearestTarget(m_chassis, m_cameras,99));
            //driver.leftBumper().onFalse(m_chassis.runOnce(() -> {m_chassis.setPrecisionTrue();}));
    //    driver.leftBumper().whileTrue(m_chassis.run(() -> {m_chassis.crossWheels();}));
    //    driver.leftBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 30*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
            // driver.rightBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(0, 30*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
            driver.rightBumper().whileTrue(mIntake.run(() -> {
                mIntake.set(INTAKE_PCT);
            }));
            // driver.rightBumper().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(INTAKE_BASE_POS_CONE, INTAKE_WRIST_POS_CONE);}));
            driver.rightBumper().onTrue(m_arm.runOnce(() -> {
                m_arm.setArmConeIntake();
                mIntake.set(INTAKE_PCT);
            }));
            driver.rightBumper().onFalse(mIntake.runOnce(() -> {
                mIntake.set(-0.075);
            }));
    
    //    driver.rightBumper().whileTrue(new ChassisTargetToCone(m_chassis, m_cameras));
    
            driver.povUp().onTrue(m_arm.runOnce(() -> {
                m_arm.setTalonTargets(m_arm.baseTalonTarget - 1000, m_arm.wristTalonTarget);
            }));
            driver.povDown().onTrue(m_arm.runOnce(() -> {
                m_arm.setTalonTargets(m_arm.baseTalonTarget + 1000, m_arm.wristTalonTarget);
            }));
    
            driver.povRight().whileTrue(mIntake.run(() -> {
                mIntake.set(-0.2 * INTAKE_PCT);
            }));
            driver.povRight().onFalse(mIntake.runOnce(() -> {
                mIntake.set(0.0);
            }));
            driver.povLeft().whileTrue(mIntake.run(() -> {
                mIntake.set(INTAKE_PCT);
            }));
            driver.povLeft().onFalse(mIntake.runOnce(() -> {
                mIntake.set(-0.075);
            }));
    
    //    driver.rightTrigger(0.7).onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(30*PI/180/(PI/1024/BASE_GEAR_RATIO), -50*PI/180/(PI/1024/WRIST_GEAR_RATIO));}));
            // driver.rightTrigger(0.7).onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(INTAKE_BASE_POS_CUBE, INTAKE_WRIST_POS_CUBE);}));
            driver.rightTrigger().onTrue(m_arm.runOnce(() -> {
                m_arm.setArmCubeIntake();
            }));
    //        driver.rightTrigger(0.7).whileTrue(mIntake.run(() -> {
    //            mIntake.set(INTAKE_PCT);
    //        }));
            driver.rightTrigger(0.7).whileTrue(new IntakeCube(mIntake, INTAKE_PCT));
            driver.rightTrigger(0.7).onFalse(mIntake.runOnce(() -> {
                mIntake.set(-0.2);
            }));
    
    
            // driver.leftTrigger().onTrue(m_chassis.runOnce(() -> {m_chassis.setPrecisionFalse();}));
            driver.leftTrigger(0.7).whileTrue(mIntake.run(() -> {
                if(m_arm.hasCone) {
                    mIntake.setVolts(OUTTAKE_VOLTS);
                }else{
                    mIntake.setVolts(OUTTAKE_VOLTS_CUBE);
                }
            }));
            driver.leftTrigger(0.7).onFalse(mIntake.runOnce(() -> {
                mIntake.set(-0.0);
            }));
    //        driver.leftTrigger(0.7).onFalse(m_chassis.runOnce(() -> {
    //            m_chassis.setPrecisionFalse();
    //        }));
    
            driver.back().onTrue(m_arm.runOnce(() -> {
                m_arm.setTalonTargets(CHUTE_BASE_POS, CHUTE_WRIST_POS);
            }));
            driver.back().onTrue(m_arm.runOnce(() -> {
                m_arm.hasCone = true;
            }));
            
            // driver.start().onTrue(new CubeFling(mIntake, m_arm));
    
    //        driver.y().and(driver.rightBumper()).onTrue(m_arm.runOnce(() -> {
    //            m_arm.setTalonTargets(SHELF_BASE_POS, SHELF_WRIST_POS);
    //        }));
    
            // driver.start().whileTrue(new ChassisDriveToNearestTarget(m_chassis, m_cameras, 99.0));
    
            //driver.start().whileTrue(new ChassisDriveToNearestTarget(m_chassis, m_cameras, 99.0));
            // driver.start().onTrue(m_arm.runOnce(() -> {m_arm.setTalonTargets(LOW_BASE_POS_CUBE, LOW_WRIST_POS_CUBE);}));
    //        driver.start().onTrue(m_chassis.runOnce(() -> {m_chassis.resetCustomOdoToOrigin();}));
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
