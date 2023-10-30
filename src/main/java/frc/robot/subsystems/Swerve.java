package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.nio.channels.ShutdownChannelGroupException;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Swerve extends SubsystemBase {
    // public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator estimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public boolean isUsingCones = true;
    private SwerveAutoBuilder autoBuilder = null;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        // Shuffleboard.getTab("Shot selection").add(')

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        estimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            Rotation2d.fromDegrees(gyro.getYaw()), 
            getModulePositions(), 
            getPose(),           
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        // return swerveOdometry.getPoseMeters();
        return estimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        // swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        estimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    
    public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if(isFirstPath){
                        this.resetOdometry(trajectory.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        trajectory,
                        this::getPose, // Pose supplier
                        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                        new PIDController(15, 0, 0.01), // X controller. Tune these values for your robot. Leaving them 0 will only use feed forwards.
                        new PIDController(15, 0, 0.01), // Y controller (usually the same values as X controller)
                        new PIDController(5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feed forwards.
                        this::setModuleStates, // Module states consumer
                        true,
                        this // Requires this drive subsystem
                )
        );
    }

    public SwerveAutoBuilder getAutoBuilder () {
        if (autoBuilder == null) {
            autoBuilder = new SwerveAutoBuilder(
                this::getPose,
                this::resetOdometry,
                Constants.Swerve.swerveKinematics,
                new PIDConstants(15, 0, 0.01),
                new PIDConstants(5, 0, 0),
                this::setModuleStates,
                Constants.AutoConstants.eventMap,
                true,
                this
            );
        } 
        return autoBuilder;
    }

    public void lock () {
        // lock pose to make it hard to move
        setModuleStates(
            new SwerveModuleState[] {
                new SwerveModuleState(0.3, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.3, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0.3, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0.3, Rotation2d.fromDegrees(45)),
            }
        );
    }
    public void stop(){
        //stop moving  -- untested
        setModuleStates(
            new SwerveModuleState[]{
                new SwerveModuleState(0.0,getYaw()),
                new SwerveModuleState(0.0,getYaw()),
                new SwerveModuleState(0.0,getYaw()),
                new SwerveModuleState(0.0,getYaw())
            }
        );
    }
    public double getRotationDegrees(){
        return getPose().getRotation().getDegrees();
    }
    
    @Override
    public void periodic(){
        // swerveOdometry.update(getYaw(), getModulePositions());  
        estimator.update(getYaw(), getModulePositions());  

        // System.out.println(field2d.getRobotPose().getX());

        // for(SwerveModule mod : mSwerveMods){
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        // }
    }
}