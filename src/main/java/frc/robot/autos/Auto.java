// package frc.robot.autos;
// import java.util.ArrayList;
// import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import com.pathplanner.lib.PathPoint;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.autos.AutoHelpers.AutoUtil;
// // import frc.robot.loggedSubsystems.Vision.Vision2;
// import frc.robot.autos.AutoHelpers.Point;
// // import frc.robot.rewrittenCommands.driving.LimelightDriveToDistance;
// // import frc.robot.subsystems.Lights;
// // import frc.robot.Util;

// /** *
//  * The class used to build and execute all of our autos. Look into the constructors to see how to use.
// */
// public class Auto {
//     private List<Point> points = new ArrayList<Point>();
//     private String autoName;
//     public PathPlannerTrajectory path;
    
//     // The goal of the autoState variable is to allow us to make a simple if statement in the Run() based for running the autos based on the constructor used
//     public AutoStates autoState = AutoStates.NO_STATE;
//     public Command compiledAuto;

//     private Swerve swerve;

//     public boolean isFinished = false;
//     public boolean hasRan = false;
//     private Command finishedCommand;

//     /**
//      * A more general overload for if there is a command from somewhere else that you want to run specifically, and i guess want the code to look cleaner. Don't see much use for this yet.
//      * 
//      * @param commandToRun - a command that will be executed
//      */
//     public Auto(Command commandToRun, Swerve swerve){
//         this.swerve = swerve;
//         compiledAuto = commandToRun;
//         autoState = AutoStates.GENERAL;
//     }

//     /***
//      * An overload that takes in a name, referencing to a pre-built Path Planner .path file, and builds an entire auto command that is executed with the Run() function
//      * 
//      * @param swerve - a reference to the swerve subsystem
//      * @param autoName - a name that matches the pre-built .path file in the 'deploy' dir
//      * 
//      * @apiNote Make sure the names match on the input and on the .path file. If they do not match it will not work.
//     */
//     public Auto(Swerve swerve, String autoName) {

//         this.swerve = swerve;
//         this.autoName = autoName;

//         GeneralLog("Starting build for auto " + autoName + ". Using PathGroup");


//         Timer timer = new Timer();
//         timer.start();

//         // currently tuning a step function
//         List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoName, new PathConstraints(4, 2));

//         GeneralLog("Path loading took " + timer.get() + "sec");

//         SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//             swerve::getPose,
//             swerve::resetOdometry,
//             // new PIDConstants(0.5, 0, 0), // open loop translation
//             // new PIDConstants(6, 0, 0), // open loop rotation
//             new PIDConstants(4.5, 0, 0), 
//             new PIDConstants(3, 0, 0.2),
//             swerve::setChassisSpeeds,
//             Constants.AutoConstants.eventMap,
//             true,
//             swerve
//         );

//         compiledAuto = autoBuilder.fullAuto(pathGroup);
//         autoState = AutoStates.AUTO_BUILDER;

//         timer.stop();
//     }

//     /**
//      * 
//      * An overload that takes in a custom list of points.
//      * 
//      * @apiNote Should only use this to adapt old auto code, not make new autos.
//      * 
//      * @param points - the list of points for the auto to follow
//      * @param swerve - a ref to the swerve subsystem (only swerve for now)
//      * @param autoName - name of the auto (not vital, only for organization)
//      */
//     public Auto (List<Point> i_points, Swerve i_swerve, String i_autoName) {
//         points = i_points;
//         swerve = i_swerve;
//         autoName = i_autoName;

//         Timer timer = new Timer();
//         timer.start();

//         path = AutoUtil.CalculatePathsFromPoints(points);
//         autoState = AutoStates.CUSTOM_POINTS;

//         GeneralLog("Path calculation took " + timer.get() + "sec");

//         compiledAuto = new PPSwerveControllerCommand(
//             path, 
//             swerve::getPose, 
//             Constants.Swerve.swerveKinematics, 
//             new PIDController(3, 0, 0), 
//             new PIDController(3, 0, 0), 
//             new PIDController(3, 0, 0),
//             swerve::setModuleStates, 
//             true, 
//             swerve
//         );
            
//         GeneralLog("Auto construction took " + timer.get() + "sec");

//         timer.stop();
//     }


//     /**
//      * The Run function for all autos. Once the auto constructor has been called with the right details, call the Run function to stat the auto.
//      * 
//      * @apiNote Designed to be only involved once. Invoking more than once will lead to problems
//     **/
//     public void Run() {
//         if(autoState == AutoStates.NO_STATE) {
//             GeneralLog("No auto compiled. Nothing running.");
//             return;
//         } else {
//             if(!hasRan) {
//                 finishedCommand = compiledAuto // get our command
//                     .beforeStarting(() -> AutoStart()) // Run the pre-auto things before we drive
//                     .andThen(Commands.runOnce(() -> AutoFinished())); // add on our auto finished function when we finish driving

//                 hasRan = true;
//             }
//             finishedCommand.schedule(); // start out command
//         }
//     }

//     public void AutoStart() {
//         System.out.println("Auto Sequence Started! Running auto: " + autoName);
//         // should only execute once
//         // lights.enableRainbow = true;
//     }
    
//     public void AutoFinished() {
//         swerve.lock();
//         // swerve.setMotorBrake(false);
//         finishedCommand.cancel();
//         GeneralLog("Auto finished. All happy here :)");
//         isFinished = true;
//         hasRan = true;
//     }

//     private void GeneralLog(String toLog) {
//         System.out.println(toLog);
//     }

//     private enum AutoStates {
//         NO_STATE,
//         GENERAL,
//         CUSTOM_POINTS,
//         AUTO_BUILDER
//     }
// }

