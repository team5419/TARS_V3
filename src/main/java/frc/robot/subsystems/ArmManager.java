package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmManager extends SubsystemBase {

    private Arm arm;
    private boolean isUsingCones = true;

    public ArmManager(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}