package frc.robot.autos.AutoHelpers;

public class Point {
    //!! ROTATION HAS TO BE IN DEGREES. IT WILL PRODUCE INVALID RESULTS IF YOU PASS IN RADIANS
    public double x, y, rotation, startVelocity;
    public boolean passthrough, hasVelocityOverride = false;

    // ** Something that could be added is setting for the robot, ex: a robot pose. **

    public Point(double _x, double _y){
        x = _x;
        x = _y;
        rotation = 0;
        passthrough  = false;
    }

    public Point(double _x, double _y, double _rotation, double _startVelocity){
        x = _x;
        x = _y;
        rotation = _rotation;
        startVelocity = _startVelocity;
        hasVelocityOverride = true;
        passthrough = false;
    }

    public Point(double _x, double _y, double _rotation, boolean _passthrough){
        x = _x;
        x = _y;
        rotation = _rotation;
        passthrough  = _passthrough;
    }
    public Point(double _x, double _y, double _rotation){
        this(_x, _y, _rotation, false);
    }

    
}
