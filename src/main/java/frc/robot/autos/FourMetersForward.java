// package frc.robot.autos;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.Swerve;

// public class FourMetersForward extends SequentialCommandGroup {

//     Swerve mChassis;
//     PathPlannerTrajectory fourMetersForward;
//     boolean isRed = DriverStation.getAlliance() == Alliance.Red;

//     public FourMetersForward(Swerve chassis) {
//         mChassis = chassis;

//         fourMetersForward = PathPlanner.loadPath("FourMetersRight", new PathConstraints(5,1));

//         addRequirements(mChassis);

//         if (isRed){
//             fourMetersForward = PathPlannerTrajectory.transformTrajectoryForAlliance(fourMetersForward, DriverStation.Alliance.Red);
//         }

//         addCommands(
//             mChassis.runOnce(() -> { mChassis.zeroGyro(); }),
//             mChassis.followTrajectoryCommand(fourMetersForward, true)
//         );

//     }

// }
