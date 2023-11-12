package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmWaypoints;
import frc.robot.subsystems.arm.GraphStator;
import frc.robot.subsystems.arm.OptimizedArm;
import frc.robot.subsystems.arm.Waypoint;

/**
 * @author Grayson / Ryan
 */
public class OptimizedMoveV2 extends CommandBase {

    private final OptimizedArm arm;
    private ArmTargets target;
    private ArmWaypoints[] waypoints;

    private int waypointIndex;
    private int numWaypoints;

    public OptimizedMoveV2(OptimizedArm arm, ArmTargets target) {
        this.arm = arm;        
        this.target = target;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.stop(); // Stop the arm
        arm.resetMotionMagic(); // Reset the motion magic

        waypointIndex = 0;

        waypoints = GraphStator.tracePath(
            new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()), 
            target
        );

        numWaypoints = waypoints.length;

        if(numWaypoints != 0 && waypoints[0] == null) { // This is only true if moving into invalid space
            System.err.println("[OPTIMIZED MOVE] Invalid position requested, abandoning move");
            this.cancel();
            return;
        } 
        // else {
        //     arm.setBicep(ControlMode.MotionMagic, waypoints[0].point.bicep);
        //     arm.setWrist(ControlMode.MotionMagic, waypoints[0].point.wrist);
        // }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ArmWaypoints currentPoint = waypoints[waypointIndex];
        if(waypointIndex == 0 || arm.isAtWithEpsilon(currentPoint, 2000)) {
            arm.configMotionMagic(
                GraphStator.calculateNewMotionMagic(
                    new Waypoint(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()), 
                    new Waypoint(currentPoint.point.bicep, currentPoint.point.wrist), 
                    arm, false
                )
            );

            arm.setBicep(ControlMode.MotionMagic, currentPoint.point.bicep);
            arm.setWrist(ControlMode.MotionMagic, currentPoint.point.wrist);

            waypointIndex++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            arm.stop();
        }

        arm.resetMotionMagic();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return waypointIndex > numWaypoints && arm.isAt(target.bicepTarget, target.wristTarget); // Grater than? RLY?
    }

}