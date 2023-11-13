package frc.robot.autos;

import frc.robot.autos.AutoHelpers.Point;
import java.util.List;

public class CustomPath {
  private int index;
  List<Point> points; // linnked list?

  public CustomPath(List<Point> points) {
    this.points = points;
  }
  // this cant be best way...
  public Point next() {
    if (index == points.size()) {
      return null;
    }
    Point rPoint = points.get(index);
    index++;
    return rPoint;
  }

  public List<Point> getPoints() {
    return points;
  }

  public Point current() {
    return points.get(index);
  }
}
