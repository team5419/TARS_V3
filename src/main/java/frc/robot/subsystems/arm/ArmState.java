package frc.robot.subsystems.arm;

public class ArmState {
  public double bicep; // x
  public double wrist; // y

  public ArmState(double bicep, double wrist) {
    this.bicep = bicep;
    this.wrist = wrist;
  }

  public ArmState toMotorState() {
    return new ArmState(
        OptimizedArm.ticksToDegreesBicep(bicep), OptimizedArm.ticksToDegreesWrist(wrist));
  }
}
