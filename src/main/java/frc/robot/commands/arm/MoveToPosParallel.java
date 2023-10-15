package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.Arm;

public class MoveToPosParallel extends CommandBase {

    private final Arm arm;
    private boolean canExit, hasFinishedFistSet;
    private double bicepTarget, wristTarget, safeZone;

    private SequentialCommandGroup group;

    /**
     * Move the arm to a position, but in parallel so it takes less time
     * @param arm - The arm
     * @param bicepTarget - The target bicep values, in ticks
     * @param wristTarget - The target wrist values, in ticks
     * @param safeZone - the min number of ticks that the bicep has to be at to allow the wrist to start moving
     * @apiNote Should not use, unless you are completely sure that the arm wont hit itself
     * 
     */
    public MoveToPosParallel(Arm arm, double bicepTarget, double wristTarget, double safeZone) {
        this.arm = arm;
        this.bicepTarget = bicepTarget;
        this.wristTarget = wristTarget;
        this.safeZone = safeZone;
    }

    /**
     * Move the arm to a position, but in parallel so it takes less time
     * @param arm - The arm
     * @param target - An ArmTargets variable with the desired arm targets
     */
    public MoveToPosParallel(Arm arm, ArmTargets target) {
        this.arm = arm;
        bicepTarget = target.bicepTarget;
        wristTarget = target.wristTarget;
        safeZone = target.safeZone;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        canExit = false;
        hasFinishedFistSet = false;
        
        // If we are going through the safe zone, or ending there, we should fist fold in the wrist so we don't hit ourself
        boolean isGoingToSameSide = Math.signum(bicepTarget) == Math.signum(arm.getBasePos()); // Same sign --> same side; as 0 is upright on both

        // boolean isGoingToSameSide = (bicepTarget <= 0) && (arm.getBaseTalonPosition() <= 0);
        // System.out.println("GOING TO SAME SIDE: " + Math.signum(arm.getBaseTalonPosition()) + " - " + Math.signum(bicepTarget) + " -- " + isGoingToSameSide + " -------------------------------------------------------------------------");

        group = new SequentialCommandGroup( // Run in sequence, to protect the bot
            new WristToPos(arm, 0).unless(() ->(Math.abs(bicepTarget) > Math.abs(safeZone) || Math.abs(arm.getBaseTalonPosition()) > Math.abs(safeZone)) && isGoingToSameSide), // Should skip if it is ending out of the safe zone and on the same side of the bot
            new InstantCommand(() -> hasFinishedFistSet = true) // B.C. the command wont end -- yes I know its bad but I don't have any time
        );
        
        group.schedule();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (hasFinishedFistSet) { // If our wrist is safe
            new BicepToPos(arm, bicepTarget).schedule(); // Start the bicep move command, that will need to happen no matter what
            if(Math.abs(arm.getBasePos()) > Math.abs(safeZone)) { // If we have cleared the safe zone 
                new WristToPos(arm, wristTarget).schedule(); // Start the wrist command
                canExit = true; // Leave this command, as the wrist and bicep will happen anyways
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return canExit;
    }

}

