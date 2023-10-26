package frc.robot.autos;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.AutoHelpers.AutoUtil;
import frc.robot.autos.AutoHelpers.Point;

/**
 * The class used to build and execute all of our autos. Look into the constructors to see how to use.
*/
public class Auto {
    private List<Point> points = new ArrayList<Point>();
    private String autoName;
    public PathPlannerTrajectory path;
    
    // The goal of the autoState variable is to allow us to make a simple if statement in the Run() based for running the autos based on the constructor used
    public AutoStates autoState = AutoStates.NO_STATE;
    public Command compiledAuto;

    private Swerve swerve;

    public boolean isFinished = false;
    private Command finishedCommand;

    /**
     * A more general overload for if there is a command from somewhere else that you want to run specifically, and i guess want the code to look cleaner. Don't see much use for this yet.
     * 
     * @param commandToRun - a command that will be executed
     */
    public Auto(Command commandToRun, Swerve swerve){
        this.swerve = swerve;
        compiledAuto = commandToRun;
        autoState = AutoStates.GENERAL;
    }

    /***
     * An overload that takes in a name, referencing to a pre-built Path Planner .path file, and builds an entire auto command that is executed with the Run() function
     * 
     * @param swerve - a reference to the swerve subsystem
     * @param autoName - a name that matches the pre-built .path file in the 'deploy' dir
     * 
     * @apiNote Make sure the names match on the input and on the .path file. If they do not match it will not work.
    */
    public Auto(Swerve swerve, String autoName) {
        this.swerve = swerve;
        this.autoName = autoName;

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoName, new PathConstraints(5, 1));

        compiledAuto = swerve.getAutoBuilder().fullAuto(pathGroup);
        compiledAuto = compiledAuto.beforeStarting(() -> start()).andThen(Commands.runOnce(() -> end()));
        autoState = AutoStates.AUTO_BUILDER;
    }

    /**
     * 
     * An overload that takes in a custom list of points.
     * 
     * @apiNote Should only use this to adapt old auto code, not make new autos.
     * 
     * @param points - the list of points for the auto to follow
     * @param swerve - a ref to the swerve subsystem (only swerve for now)
     * @param autoName - name of the auto (not vital, only for organization)
     */
    public Auto (List<Point> i_points, Swerve i_swerve, String i_autoName) {
        points = i_points;
        swerve = i_swerve;
        autoName = i_autoName;

        path = AutoUtil.CalculatePathsFromPoints(points);
        autoState = AutoStates.CUSTOM_POINTS;

        compiledAuto = swerve.followTrajectoryCommand(path, true);     
        
        // compiledAuto.beforeStarting(() -> AutoStart()).andThen(Commands.runOnce(() -> AutoFinished()));
    }


    /**
     * The Run function for all autos. Once the auto constructor has been called with the right details, call the Run function to stat the auto.
     * 
     * @apiNote Designed to be only involved once. Invoking more than once will lead to problems
    **/
    public void Run() {
        if(autoState == AutoStates.NO_STATE) {
            GeneralLog("No auto compiled. Nothing running.");
            return;
        }

        compiledAuto.schedule(); // start out command
    }

    public void start() {
        System.out.println("Auto Sequence Started! Running auto: " + autoName);
    }
    
    public void end() {
        swerve.lock();
        compiledAuto.cancel();
        GeneralLog("Auto finished. All happy here :)");
        isFinished = true;
    }

    private void GeneralLog(String toLog) {
        System.out.println(toLog);
    }

    private enum AutoStates {
        NO_STATE,
        GENERAL,
        CUSTOM_POINTS,
        AUTO_BUILDER
    }
}

