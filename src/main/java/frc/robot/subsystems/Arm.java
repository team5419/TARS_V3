package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

public class Arm extends SubsystemBase {

    public double baseTalonTarget = 1000;
    public double wristTalonTarget = 0;

    public WPI_TalonFX baseTalon;
    public WPI_TalonFX wristTalon;

    public WPI_TalonFX baseFollower;

    // keep false please - testing ONLY
    public boolean keepParallel = false;

    public boolean hasCone = true;

    public boolean overriding = false;

    private ShuffleboardTab tab = Shuffleboard.getTab("General");

    public GenericEntry midBaseFudgeTab = tab.add("Mid Score Bicep (Degrees)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
//            .withProperties(Map.of("min", -30, "max", 30)).getEntry();
            .withProperties(Map.of("min", -5, "max", 5)).getEntry();
    public GenericEntry midWristFudgeTab = tab.add("Mid Score Forearm (Degrees)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            // .withProperties(Map.of("min", -30, "max", 30)).getEntry();
            .withProperties(Map.of("min", -5, "max", 5)).getEntry();

    public GenericEntry highBaseFudgeTab = tab.add("High Score Bicep (Degrees)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
//            .withProperties(Map.of("min", -30, "max", 30)).getEntry();
            .withProperties(Map.of("min", -5, "max", 5)).getEntry();
    public GenericEntry highWristFudgeTab = tab.add("High Score Forearm (Degrees)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            // .withProperties(Map.of("min", -30, "max", 30)).getEntry();
            .withProperties(Map.of("min", -5, "max", 5)).getEntry();

    public GenericEntry coneBaseFudgeTab = tab.add("Cone Intake Bicep (Degrees)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            // .withProperties(Map.of("min", -30, "max", 30)).getEntry();
            .withProperties(Map.of("min", -5, "max", 5)).getEntry();
    public GenericEntry coneWristFudgeTab = tab.add("Cone Intake Forearm (Degrees)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            // .withProperties(Map.of("min", -30, "max", 30)).getEntry();
            .withProperties(Map.of("min", -5, "max", 5)).getEntry();

    public GenericEntry cubeBaseFudgeTab = tab.add("Cube Intake Bicep (Degrees)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            // .withProperties(Map.of("min", -30, "max", 30)).getEntry();
            .withProperties(Map.of("min", -5, "max", 5)).getEntry();
    public GenericEntry cubeWristFudgeTab = tab.add("Cube Intake Forearm (Degrees)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            // .withProperties(Map.of("min", -30, "max", 30)).getEntry();
            .withProperties(Map.of("min", -5, "max", 5)).getEntry();


    public Arm(boolean tuningMode) {
        baseTalon = new WPI_TalonFX(BASE_ID);
        wristTalon = new WPI_TalonFX(WRIST_ID);

        baseFollower = new WPI_TalonFX(BASE_FOLLOWER_ID);

        baseTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 20, 0.1));
        baseFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 20, 0.1));
        wristTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 20, 0.1));

        baseTalon.setSelectedSensorPosition(BASE_START_POS);

        baseTalon.setNeutralMode(tuningMode ? NeutralMode.Coast : NeutralMode.Brake);

        baseTalon.config_kF(0, BASE_kF);
        baseTalon.config_kP(0, BASE_kP);
        baseTalon.config_kI(0, BASE_kI);
        baseTalon.config_kD(0, BASE_kD);

        baseTalon.configMotionCruiseVelocity(BASE_MAX_V);
        // baseTalon.configMotionAcceleration(BASE_MAX_A);
        
        // baseTalon.configMotionCruiseVelocity(12000);
        baseTalon.configMotionAcceleration(40000);

        baseTalon.configMotionSCurveStrength(BASE_CURVE_STR);

        baseFollower.setNeutralMode(tuningMode ? NeutralMode.Coast : NeutralMode.Brake);
        baseFollower.follow(baseTalon);
        baseFollower.setInverted(InvertType.FollowMaster);

        wristTalon.setSelectedSensorPosition(WRIST_START_POS);

        wristTalon.setInverted(TalonFXInvertType.Clockwise);
        wristTalon.setNeutralMode(tuningMode ? NeutralMode.Coast : NeutralMode.Brake);

        wristTalon.config_kF(0, WRIST_kF);
        wristTalon.config_kP(0, WRIST_kP);
        wristTalon.config_kI(0, WRIST_kI);
        wristTalon.config_kD(0, WRIST_kD);

        wristTalon.configMotionCruiseVelocity(WRIST_MAX_V);
        wristTalon.configMotionAcceleration(WRIST_MAX_A);
        // wristTalon.configMotionAcceleration(40000);
        
        wristTalon.configMotionSCurveStrength(WRIST_CURVE_STR);

        highBaseFudgeTab.setDouble(0.0);
        highWristFudgeTab.setDouble(0.0);

        midBaseFudgeTab.setDouble(0.0);
        midWristFudgeTab.setDouble(0.0);

        coneBaseFudgeTab.setDouble(0.0);
        coneWristFudgeTab.setDouble(0.0);

        cubeBaseFudgeTab.setDouble(0.0);
        cubeWristFudgeTab.setDouble(0.0);    
    }

//    @Override
   public void periodic() {
        // System.out.println(wristTalon.getClosedLoopTarget());
        SmartDashboard.putNumber("Bicep Encoder", baseTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("Wrist Encoder", wristTalon.getSelectedSensorPosition());
   }

   public double degreesToTicks(double degrees) {
        return degrees * PI / 180 / (PI / 1024 / BASE_GEAR_RATIO);
   }

    public void passSetpoints(double basePos, double wristPos) {
        // baseTalon.set(TalonFXControlMode.MotionMagic, basePos);
        // wristTalon.set(TalonFXControlMode.MotionMagic, wristPos);
    }

    public double getBaseTalonPosition() {
        return baseTalon.getSelectedSensorPosition();
    }

    // might be something to investigate?
    // see WPILib arm tutorial for how to calc FF.
    public void passSetpointsFF(int basePos, int wristPos) {
        // double baseAngleRad = baseTalon.getSelectedSensorPosition()*PI/1024/BASE_GEAR_RATIO;
        // double wristAngleRad = wristTalon.getSelectedSensorPosition()*PI/1024/WRIST_GEAR_RATIO;

        // double baseFFAdj = BASE_FF*cos(baseAngleRad) + WRIST_FF*cos(baseAngleRad+wristAngleRad);
        // double wristFFAdj = WRIST_FF*cos(baseAngleRad+wristAngleRad);

        // baseTalon.set(TalonFXControlMode.Position, basePos, DemandType.ArbitraryFeedForward, baseFFAdj);
        // wristTalon.set(TalonFXControlMode.Position, wristPos, DemandType.ArbitraryFeedForward, wristFFAdj);
    }

    // path target
    public void setTalonTargets(double baseTarget, double wristTarget) {
        // baseTalonTarget = baseTarget;
        // wristTalonTarget = wristTarget;
    }

    public boolean armAtTarget() {
        return (abs(baseTalon.getSelectedSensorPosition() - baseTalonTarget) < BASE_ERROR_THRESHOLD);
    }

    public boolean wristAtTarget() {
        return (abs(wristTalon.getSelectedSensorPosition() - wristTalonTarget) < BASE_ERROR_THRESHOLD);
    }

    // check if ok to move both arms at once
    public boolean safeForDouble() {
        return (abs(baseTalon.getSelectedSensorPosition()) > BASE_SAFETY_THRESHOLD);
    }

    // checks if wrist is in the arm or not
    public boolean wristStowed() {
        return (abs(wristTalon.getSelectedSensorPosition() - WRIST_STOWED_POS) < WRIST_ERROR_THRESHOLD);
    }

    public void stowWrist() {
        baseTalon.set(TalonFXControlMode.PercentOutput, 0);
        wristTalon.set(TalonFXControlMode.MotionMagic, WRIST_STOWED_POS);
    }

    public void setMotorsBrake() {
        baseTalon.set(0);
        wristTalon.set(0);
    }

    public void setBaseBrake() {
        baseTalon.set(0);
    }

    public void setHoldCurrentPos() {
        baseTalonTarget = baseTalon.getSelectedSensorPosition();
        wristTalonTarget = wristTalon.getSelectedSensorPosition();
    }

    // the +/- is different on the fudges because I want positive to mean against gravity which is more intuitive then mapping
    // one side to be up and other to be down on the fly
    public void setArmHigh() {
        // overriding = false;

        // // BASE/WRIST_CONVERSION_FACTOR instead of 1000
        // baseTalonTarget = HIGH_BASE_POS - highBaseFudgeTab.getDouble(0.0)*1000;
        // wristTalonTarget = HIGH_WRIST_POS - highWristFudgeTab.getDouble(0.0)*1000;
    }

    public double getHighBaseFudge() {
        return highBaseFudgeTab.getDouble(0.0)*1000;
    }

    public double getHighWristFudge() {
        return highBaseFudgeTab.getDouble(0.0)*1000;
    }

    public void setArmMid() {
        // overriding = false;
        // if (hasCone) {
        //     baseTalonTarget = MID_BASE_POS - midBaseFudgeTab.getDouble(0.0) * 1000;
        //     wristTalonTarget = MID_WRIST_POS_TELE - midWristFudgeTab.getDouble(0.0) * 1000;
        // } else {
        //     baseTalonTarget = MID_BASE_POS_CUBE - midBaseFudgeTab.getDouble(0.0) * 1000;
        //     wristTalonTarget = MID_WRIST_POS_CUBE - midWristFudgeTab.getDouble(0.0) * 1000;
        // }
    }

    public void setArmConeIntake() {
        // overriding = false;
        // baseTalonTarget = INTAKE_BASE_POS_CONE + coneBaseFudgeTab.getDouble(0.0)*1000;
        // wristTalonTarget = INTAKE_WRIST_POS_CONE + coneWristFudgeTab.getDouble(0.0)*1000;

        // hasCone = true;
    }

    public void setArmCubeIntake() {
        // overriding = false;
        // baseTalonTarget = INTAKE_BASE_POS_CUBE + cubeBaseFudgeTab.getDouble(0.0)*1000;
        // wristTalonTarget = INTAKE_WRIST_POS_CUBE + cubeWristFudgeTab.getDouble(0.0)*1000;

        // hasCone = false;
    }

    public void setArmStow() {
        // overriding = false;

        // baseTalonTarget = 0;
        // wristTalonTarget = 0;
        // passSetpoints(baseTalonTarget, wristTalonTarget);
    }

    // convenience function to see if arm is at a position yet
    public boolean isAt(double basePos, double wristPos) {
        return (abs(basePos - baseTalon.getSelectedSensorPosition()) < 1000)
                && (abs(wristPos - wristTalon.getSelectedSensorPosition()) < 1000);
    }

    public double getBasePos() {
        return baseTalon.getSelectedSensorPosition();
    }

    public double getWristPos() {
        return wristTalon.getSelectedSensorPosition();
    }

    public void configBaseAccel(double accelLimit) {
        baseTalon.configMotionAcceleration(accelLimit);
    }

}
