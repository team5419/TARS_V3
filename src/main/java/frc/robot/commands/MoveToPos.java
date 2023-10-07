package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class MoveToPos extends CommandBase {

    private final Arm arm;

    private double bicepTarget = 0;
    private double wristTarget = 0;

    private SequentialCommandGroup group;

    public MoveToPos(Arm arm, double bicepTarget, double wristTarget) {
        this.arm = arm;
        this.bicepTarget = bicepTarget;
        this.wristTarget = wristTarget;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        group = new SequentialCommandGroup(
            new WristToPos(arm, 0),
            new BisepToPos(arm, bicepTarget),
            new WristToPos(arm, wristTarget)
        );
        group.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        System.out.println(group.isFinished());
        return group.isFinished();
    }

}