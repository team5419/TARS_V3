// package frc.robot.commands.arm;

// import com.ctre.phoenix.motorcontrol.ControlMode;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.ArmTargets;
// import frc.robot.subsystems.arm.ArmState;
// import frc.robot.subsystems.arm.ArmWaypoints;
// import frc.robot.subsystems.arm.GraphStator;
// import frc.robot.subsystems.arm.OptimizedArm;
// import frc.robot.subsystems.arm.Waypoint;

// /**
//  * @author Grayson / Ryan
//  */
// public class OptimizedMoveV2 extends CommandBase {

//     private final OptimizedArm arm;
//     private ArmTargets target;
//     private ArmWaypoints[] waypoints;
//     private ArmWaypoints currentPoint;

//     private int waypointIndex;
//     private int numWaypoints;
//     private boolean isDone;

//     public OptimizedMoveV2(OptimizedArm arm, ArmTargets target) {
//         this.arm = arm;        
//         this.target = target;

//         // Use addRequirements() here to declare subsystem dependencies.
//         addRequirements(arm);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         arm.stop(); // Stop the arm
//         arm.resetMotionMagic(); // Reset the motion magic

//         waypointIndex = 0;
//         isDone = false;

//         waypoints = GraphStator.tracePath(
//             new ArmState(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()), 
//             target
//         );

//         numWaypoints = waypoints.length;

//         if(numWaypoints == 0) { // This is only true if moving into invalid space
//             configAndMoveTo(new Waypoint(target.bicepTarget, target.wristTarget));
//             isDone = true;
//             return;
//         } else if (numWaypoints != 0 && waypoints[0] == null) {
//             System.err.println("[OPTIMIZED MOVE] Invalid position requested, abandoning move");
//             this.cancel();
//             return;
//         } else {
//             configAndMoveTo(waypoints[0]);
//         }
        
//         currentPoint = waypoints[0];

//         System.out.println("Number points " + numWaypoints);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         System.out.println("Arm is at epsilon " + arm.isAtWithEpsilon(currentPoint, 2000));
//         if(!isDone && arm.isAtWithEpsilon(currentPoint, 2000)) { 
//             waypointIndex++;
//             System.out.println("Going to waypoint " + waypointIndex);
//             if(waypointIndex < numWaypoints) {
//                 currentPoint = waypoints[waypointIndex];
//                 configAndMoveTo(currentPoint);
//             } else {
//                 Waypoint finalWaypoint = new Waypoint(target.bicepTarget, target.wristTarget);
//                 configAndMoveTo(finalWaypoint);
//                 isDone = true;
//             }
//         }
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         if(interrupted) {
//             arm.stop();
//         }

//         arm.resetMotionMagic();

//         System.out.println("EXITED -------------------------------------------------------------------------------------------");
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return isDone && arm.isAtWithEpsilon(target, 2000); // Grater than? RLY?
//     }

//     private void configAndMoveTo (ArmWaypoints point) {
//         arm.configMotionMagic(
//             GraphStator.calculateNewMotionMagic(
//                 new Waypoint(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()), 
//                 new Waypoint(OptimizedArm.degreesToTicksBicep(point.point.bicep), OptimizedArm.degreesToTicksBicep(point.point.wrist)), 
//                 arm, true
//             )
//     );
    
//         arm.setBicep(ControlMode.MotionMagic, OptimizedArm.degreesToTicksBicep(point.point.bicep));
//         arm.setWrist(ControlMode.MotionMagic, OptimizedArm.degreesToTicksBicep(point.point.wrist));
//     }

//     private void configAndMoveTo (Waypoint point) {
//         arm.configMotionMagic(
//                     GraphStator.calculateNewMotionMagic(
//                         new Waypoint(arm.getBicepPositionDegrees(), arm.getWristPositionDegrees()), 
//                         new Waypoint(point.bicep, point.wrist), 
//                         arm, true
//                     )
//                 );
    
//         arm.setBicep(ControlMode.MotionMagic, point.bicep);
//         arm.setWrist(ControlMode.MotionMagic, point.wrist);
//     }

// }