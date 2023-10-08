package frc.robot.autos;

import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.MoveToPos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class OnePieceAuto extends SequentialCommandGroup{
    public OnePieceAuto(Swerve swerve, Arm arm, Intake intake){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        Trajectory outOfCommunityTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(5, 0)),
                new Pose2d(3, 0, new Rotation2d(0)),
                config);
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        SwerveControllerCommand swerveControllerCommand = 
        new SwerveControllerCommand(
                outOfCommunityTrajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);
        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(outOfCommunityTrajectory.getInitialPose())),
            new MoveToPos(arm,  Constants.ArmConstants.HIGH_BASE_POS_ALT, Constants.ArmConstants.HIGH_WRIST_POS_ALT),
            new InstantCommand(() -> intake.set(-0.1)),
            new MoveToPos(arm, 0, 0),
            swerveControllerCommand
        );
        
    }
}