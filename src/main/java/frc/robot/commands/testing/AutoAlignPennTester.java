package frc.robot.commands.testing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.autos.AutoHelpers.Point;
import frc.robot.autos.CustomPath;
import java.util.ArrayList;
import java.util.List;

public class AutoAlignPennTester {

  private final PIDController horizontalController;
  private final PIDController veritcalController;
  private final ProfiledPIDController rotationController;

  public AutoAlignPennTester() {
    this.horizontalController = new PIDController(0.1, 0, 0); //untuned
    this.veritcalController = new PIDController(0.1, 0, 0); //untuned

    this.rotationController =
      new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0,
        Constants.AutoConstants.kThetaControllerConstraints
      );
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    testPIDs();
  }

  public void testPIDs() {
    CustomPath path = testDirectPath();
    Point target = path.next();

    double newHorizontal = horizontalController.calculate(
      simulateTX(),
      15
    );
    // double newVertical = veritcalController.calculate(simulateTY(), 0.0);

  }

  public CustomPath testDirectPath() {
    List<Point> points = new ArrayList<>();
    //not sure if array indexes are correct
    points.add(
      new Point(simulateHorizontalDistance(), simulateVerticalDistance(), 0)
    );
    return new CustomPath(points);
  }
  public double simulateHorizontalDistance(){
    return 15;
  }
  public double simulateVerticalDistance(){
    return -5;
  }
  public double simulateTX(){
    return -5;
  }



  private Point getTarget(){
      return new Point(simulateHorizontalDistance(), simulateVerticalDistance(), 0);
  }


  public void customTwo(){
      Point target = getTarget();
      System.out.println(target);
    
        double newHorizontal = horizontalController.calculate(
          0.0,
          target.x
        );
        double newVertical = veritcalController.calculate(
          0.0, 
          target.y
        );
        // double newRotation = rotationController.calculate(
        //   swerve.getRotationDegrees(),
        //   target.rotation
        // );
      System.out.println("NEW HORIZONTAL: " + newHorizontal);
      System.out.println("NEW VERTICAL: " + newVertical);
  }
}
