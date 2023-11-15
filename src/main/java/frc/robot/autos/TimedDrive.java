// package frc.robot.autos;

// import java.util.List;
// import java.util.Set;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.commands.arm.MoveToPos;
// import frc.robot.commands.arm.TwoStageHighAuto;
// import frc.robot.commands.swerve.TimeDrive;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Swerve;

// public class TimedDrive extends SequentialCommandGroup{
//     public TimedDrive(Swerve swerve, Arm arm, Intake intake){
//         addCommands(
//             new InstantCommand(() -> swerve.zeroGyro()),
//             new InstantCommand(() -> intake.set(-0.1)),
//             new TwoStageHighAuto(arm, intake),
//             new MoveToPos(arm, 0, 0),
//             new TimeDrive(swerve, 4.5)
//         );

//     }
// }
