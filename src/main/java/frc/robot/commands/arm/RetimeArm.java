package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.arm.GraphStator.Waypoint;

/**
 * @author 
 */
public class RetimeArm extends CommandBase {

    private final OptimizedArm arm;
    private ArmTargets target;
    private Waypoint targetWaypoint;

    public RetimeArm(OptimizedArm arm, ArmTargets target) {
        this.arm = arm;        
        this.target = target;
        this.targetWaypoint = null;
    }

        public RetimeArm(OptimizedArm arm, Waypoint target) {
        this.arm = arm;        
        this.target = null;
        this.targetWaypoint = target;
    }

    @Override
    public void initialize() {
        Waypoint starWaypoint = new Waypoint(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees());
        if(target == null) {
            arm.configMotionMagic(arm.graphStator.calculateNewMotionMagic(starWaypoint, targetWaypoint));
        } else {
            Waypoint endWaypoint = new Waypoint(arm.ticksToDegreesBicep(target.bicepTarget), arm.ticksToDegreesWrist(target.wristTarget));
            arm.configMotionMagic(arm.graphStator.calculateNewMotionMagic(starWaypoint, endWaypoint));
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}