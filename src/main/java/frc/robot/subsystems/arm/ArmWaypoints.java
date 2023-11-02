package frc.robot.subsystems.arm;

public enum ArmWaypoints {
  QUAD_A(0, 0),
  QUAD_B(45, 70),
  QUAD_C(-45, 70),
  QUAD_D(-45, -55),
  QUAD_E(45, -70),
  QUAD_F(15, 60);

  public ArmState point;
  ArmWaypoints(double bicep, double wrist) {
    point = new ArmState(bicep, wrist);
  }
}