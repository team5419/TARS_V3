package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.AutoHelpers.Point;

public class Util {
  public static final double getDistance(double x, double y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public static final Pose2d pointToPose2d(Point point) {
    return new Pose2d(point.x, point.y, new Rotation2d(point.rotation));
  }
}
