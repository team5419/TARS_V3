package frc.robot.autos;

import java.util.List;

import frc.robot.autos.AutoHelpers.Point;

public class CustomPath {
    private int index;
    List<Point> points; //linnked list?
    public CustomPath(List<Point> points){
        this.points = points;
    }
    //this cant be best way...
    public Point next(){
        Point rPoint = points.get(index);
        index++;
        return rPoint;
    }
    public List<Point> getPoints(){
        return points;
    }
    public Point current(){
        return points.get(index);
    }
    
}
