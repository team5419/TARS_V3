package frc.robot.autos.AutoHelpers;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class AutoUtil {
    /**
     *  Calculates a spline from a list of points.
     *  Ideally we would not use this function for any new autos, but rather to re-adapt old autos into something that can be used quick
     * 
     * @param points - a list of points that will be used
     * 
    **/
    public static PathPlannerTrajectory CalculatePathsFromPoints(List<Point> points) {
        // Using a lib called "PathPlanner" (https://github.com/mjansen4857/pathplanne) for the curves
        // (i cant be bothered to make efficient bézier curves, and its free ¯\_(ツ)_/¯ )
        //? For this implementation specifically: we will be following the "on the fly" spline generation
        //? Docs here: https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#on-the-fly-generation
        //? Custom use of the path can be found here: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html
        //? However, you shouldn't need to do that. For example, on the SwerveDrive class there is a command builder for following a set path. 


        List<PathPoint> pointList = new ArrayList<PathPoint>();

        for (Point point: points) {
            pointList.add(
                new PathPoint(
                    new Translation2d(point.x, point.y), 
                    Rotation2d.fromDegrees(point.rotation), 
                    point.hasVelocityOverride ? point.startVelocity : 0
                )
            ); // position, heading(direction of travel), holonomic rotation, possible non-zero starting velocity
        }

        return PathPlanner.generatePath(
            new PathConstraints(Constants.Swerve.maxSpeed, 2), 
            pointList
        );
    }
}
