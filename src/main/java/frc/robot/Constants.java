package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import static java.lang.Math.PI;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 51;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26.5); 
        public static final double wheelBase = Units.inchesToMeters(26.5);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 6.0;
        /** Slow / precision mode values */
        public static final double slowModeSpeedMultiplier = 0.2;
        public static final double slowModeTurnMultiplier = 0.3;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 31;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.43);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(181.494 - 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(25.57 + 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 24;
            public static final int canCoderID = 34;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(9.580 + 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 23;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-92.109 + 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class ArmTargets {
        public double wristTarget;
        public double bicepTarget;
        public double safeZone;

        public ArmTargets() {
            this.bicepTarget = 0;
            this.wristTarget = 0;
            this.safeZone = 0;
        }

        public ArmTargets (double bicepTarget, double wristTarget) {
            this.bicepTarget = bicepTarget;
            this.wristTarget = wristTarget;
        }


        public ArmTargets (double bicepTarget, double wristTarget, double safeZone) {
            this.bicepTarget = bicepTarget;
            this.wristTarget = wristTarget;
            this.safeZone = safeZone;
        }

        public ArmTargets fromDegrees (double bicepDegrees, double wristDegrees) {
            this.bicepTarget = bicepDegrees * PI / 180 / (PI / 1024 / (58.286 * 3 / 36 * 20));
            // Some magic done by 6672                                ^^^^^^^^^^^^^^^^^^^^^^ Arm gear ratio
            this.wristTarget = wristDegrees * PI / 180 / (PI / 1024 / (54));
            // Some magic done by 6672                                ^^^^ Arm gear ratio
            this.safeZone = 0;
            // Same magic done by 6672
            return this;
        }

        public ArmTargets fromDegrees (double bicepDegrees, double wristDegrees, double safeZoneDegrees) {
            this.bicepTarget = bicepDegrees * PI / 180 / (PI / 1024 / (58.286 * 3 / 36 * 20));
            // Some magic done by 6672                                ^^^^^^^^^^^^^^^^^^^^^^ Arm gear ratio
            this.wristTarget = wristDegrees * PI / 180 / (PI / 1024 / (54));
            // Some magic done by 6672                                ^^^^ Arm gear ratio
            this.safeZone = safeZoneDegrees * PI / 180 / (PI / 1024 / (58.286 * 3 / 36 * 20));
            // Same magic done by 6672
            return this;
        }

        // public ArmTargets toDegrees ()
        
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final HashMap<String, Command> eventMap = new HashMap<String, Command>();
    }
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    // INTAKE MOTOR ID
    public static final int INTAKE_ID = 44;

    public static final double INTAKE_PCT = -0.4;

    public static final double OUTTAKE_VOLTS = 6.3;
    public static final double OUTTAKE_VOLTS_CUBE = 4.2;
  }

//   public static class AutoConstants {

//   }

  public static class ArmConstants {

    // public static final ArmTargets high = new ArmTargets(-67421.79081481483 - 2000, -45004.799999999996 + 6000); // This looks bad IK but I don't have a choice
    // public static final ArmTargets ground = new ArmTargets(12986.98 - 2000, -47616.0 + 8000);

    public static final double globalSafeZone = 37863;
    
    public static final ArmTargets stow = new ArmTargets(0, 0);
    
    public static final ArmTargets coneSubstation = new ArmTargets(2933, 10842);
    public static final ArmTargets coneHigh = new ArmTargets(-63004, -45933, globalSafeZone);
    public static final ArmTargets coneMid = new ArmTargets(-67582, -26966, globalSafeZone);
    public static final ArmTargets coneHybrid = new ArmTargets().fromDegrees(10, 70);
    public static final ArmTargets coneGround = new ArmTargets(13481, -46766, globalSafeZone);

    public static final ArmTargets cubeSubstation = coneSubstation;
    public static final ArmTargets cubeHigh = coneHigh;
    public static final ArmTargets cubeMid = new ArmTargets().fromDegrees(-15, 55, globalSafeZone);
    public static final ArmTargets cubeHybrid = new ArmTargets().fromDegrees(10, 70);
    public static final ArmTargets cubeGround = new ArmTargets().fromDegrees(50.5, -175, globalSafeZone);
    // public static final double INTAKE_BASE_POS_CUBE = 45.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    // public static final double INTAKE_WRIST_POS_CUBE = -175*PI/180/(PI/1024/WRIST_GEAR_RATIO);
    
    public static final int BASE_ID = 42;
    public static final int WRIST_ID = 43;

    public static final int BASE_FOLLOWER_ID = 41;

    public static final double BASE_kF = 0;
    public static final double BASE_kP = 0.2;
    public static final double BASE_kI = 0;
    public static final double BASE_kD = 0;

    public static final double BASE_MAX_V = 19500; // ticks / 100ms
    public static final double BASE_MAX_A = 48000; // ticks / 100ms / s
    public static final int BASE_CURVE_STR = 2; // smoothness

    public static final double WRIST_kF = 0;
    public static final double WRIST_kP = 0.4;
    public static final double WRIST_kI = 0;
    public static final double WRIST_kD = 0;

    public static final double WRIST_MAX_V = 19500;
    public static final double WRIST_MAX_A = 48000.0; // could be up to 102400 with good enough intake
    public static final int WRIST_CURVE_STR = 2;

    // slomo testing
//    public static final double WRIST_MAX_V = 2000;
//    public static final double WRIST_MAX_A = 4000;
//    public static final int WRIST_CURVE_STR = 1;

    public static final double BASE_FF = 0;
    public static final double WRIST_FF = 0.0;

    public static final double BASE_GEAR_RATIO = 58.286*3/36*20;
    public static final double WRIST_GEAR_RATIO = 54;

    public static final int BASE_START_POS = 0;
    public static final int WRIST_START_POS = 0;

    public static final int BASE_ERROR_THRESHOLD = 8000;
    public static final int WRIST_ERROR_THRESHOLD = 20000;

    public static final double BASE_SAFETY_THRESHOLD = 45*PI/180/(PI/1024/BASE_GEAR_RATIO);

    public static final int WRIST_STOWED_POS = 0;

    // degrees to motor ticks
    public static final double BASE_CONVERSION_FACTOR = PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double WRIST_CONVERSION_FACTOR = PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double MID_BASE_POS = -127*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double MID_WRIST_POS = -90*PI/180/(PI/1024/WRIST_GEAR_RATIO);
    public static final double MID_WRIST_POS_TELE = -95*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double MID_BASE_POS_CUBE = -15*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double MID_WRIST_POS_CUBE = 55*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double HIGH_BASE_POS = -131.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS = -129*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double HIGH_BASE_POS_VIKES = -131.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_VIKES = -129*PI/180/(PI/1024/WRIST_GEAR_RATIO);


    public static final double HIGH_WRIST_POS_AUTO = -113.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    // testing to get cone very close to high
    public static final double HIGH_BASE_POS_ALT_PREP = -134.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_BASE_POS_ALT = -122*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_ALT = -146.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);

//    public static final double HIGH_BASE_POS_ALT = -114*PI/180/(PI/1024/BASE_GEAR_RATIO);
//    public static final double HIGH_WRIST_POS_ALT = -161.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);

//    public static final double HIGH_BASE_POS_ALT = -121*PI/180/(PI/1024/BASE_GEAR_RATIO);
//    public static final double HIGH_WRIST_POS_ALT = -148.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double HIGH_BASE_POS_ALT_FTW = -121.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_ALT_FTW = -148.5*PI/180/(PI/1024/WRIST_GEAR_RATIO);
    public static final double HIGH_BASE_POS_ALT_WACO = -118*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_ALT_WACO = -152*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double HIGH_BASE_POS_ALT_AUTO = -120.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double HIGH_WRIST_POS_ALT_AUTO = -152*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double INTAKE_BASE_POS_CONE = 23.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double INTAKE_WRIST_POS_CONE = -155*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double INTAKE_BASE_POS_CUBE = 45.5*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double INTAKE_WRIST_POS_CUBE = -175*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double CHUTE_BASE_POS = 15*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double CHUTE_WRIST_POS = -58
              *PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double SHELF_BASE_POS = 90*PI/180/(PI/1024/BASE_GEAR_RATIO);
    public static final double SHELF_WRIST_POS = -180*PI/180/(PI/1024/WRIST_GEAR_RATIO);

//    public static final double LOW_BASE_POS_CUBE = INTAKE_BASE_POS_CUBE;
//    public static final double LOW_WRIST_POS_CUBE = 45*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double LOW_BASE_POS_CUBE = 0;
    public static final double LOW_WRIST_POS_CUBE = 70*PI/180/(PI/1024/WRIST_GEAR_RATIO);

    public static final double FLING_BASE_POS = 0;
    public static final double FLING_INIT_WRIST_POS = -50*PI/180/(PI/1024/WRIST_GEAR_RATIO);
    public static final double FLING_FINAL_WRIST_POS = 70*PI/180/(PI/1024/WRIST_GEAR_RATIO);

  }

  // MOTOR IDS for the SWERVE
  public static int AXIS_FL_ID = 8;
  public static int AXIS_BL_ID = 4;
  public static int AXIS_FR_ID = 2;
  public static int AXIS_BR_ID = 6;

  public static int DRIVE_FL_ID = 12;
  public static int DRIVE_BL_ID = 13;
  public static int DRIVE_FR_ID = 11;
  public static int DRIVE_BR_ID = 14;

  // CANCODER IDS
  public static int CODER_FL_ID = 32; //POSSIBLY 30'S
  public static int CODER_BL_ID = 33;
  public static int CODER_FR_ID = 31;
  public static int CODER_BR_ID = 34;

  public static double[] ENCODER_OFFSETS = {
    210.498, //FL
    446.484 - 360, //BL
    -2.285 + 360, //FR
    190.635, //BR

  };
  // Indexer Output
  public static double INDEXER_TARGET = 0.8;

  // Drivebase Facts
  public static double TRACK_WIDTH_METERS = 0.4953;
  public static double TRACK_LENGTH_METERS = 0.4953;

//  public static double TRACK_WIDTH_METERS = 0.7112;
//  public static double TRACK_LENGTH_METERS = 0.7112;

  public static double TRACK_WIDTH_METERS_TEST = 0.4953;
  public static double TRACK_LENGTH_METERS_TEST = 0.4953;

  public static double SWERVE_FORWARD_SPEED_MAX = 6.6;
  public static double SWERVE_STRAFE_SPEED_MAX = 6.6;
  public static double SWERVE_ROT_SPEED_MAX = 4.0 / 0.4953 * 0.7112;

  public static double MAX_SPEED = 9.2;

  // PIDs (potentially very wack) (olds can be found in 2022 code)
  public static double AXIS_kF = 0.0;
  public static double AXIS_kP = 0.15;
  public static double AXIS_kI = 0.0;
  public static double AXIS_kD = 0.1;

  // note SDS default 0 0.2 0 0.1

  // PIDs (potentially very wack)
  public static double DRIVE_kF = 0.0;
  public static double DRIVE_kP = 0.07;
  public static double DRIVE_kI = 0.0;
  public static double DRIVE_kD = 0.00;

  public static double STEERING_RATIO = 150.0/7.0;
  public static double DRIVING_RATIO = 6.75;

  public static double WHEEL_RADIUS_METERS = 0.0508;

  // AutoBalance alt constants
  public static double CHARGE_STATION_BALANCE_ANGLE_GOAL = 3.25;

  public static double AUTON_DRIVE_kP = 0.6;

  public static double CHARGE_STATION_STABILIZE_SECONDS = .1;

  public static boolean IS_LOGGING = true;
}