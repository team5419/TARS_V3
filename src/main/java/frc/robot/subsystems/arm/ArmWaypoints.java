package frc.robot.subsystems.arm;

public enum ArmWaypoints {
  QUAD_A(0, 0),
  QUAD_B(45, 70),
  QUAD_C(45, -70),
  // QUAD_D(-45, -45),
  QUAD_D(-55, -20),
  QUAD_E(
      -45, 55), // Could want to modify this to allow high shots to clear the post without hitting
  QUAD_F(25, -55);

  public ArmState point;

  ArmWaypoints(double bicep, double wrist) {
    point = new ArmState(bicep, wrist);
  }
}
