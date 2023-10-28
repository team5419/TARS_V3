package frc.robot.subsystems.arm;

import java.util.HashMap;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OptimizedArm extends SubsystemBase {
    // Our thee motors
    protected WPI_TalonFX bicepTalon;
    protected WPI_TalonFX bicepTalonFollower;
    protected WPI_TalonFX wristTalon;
    private boolean isInBrakeMode;

    public ArmState graphStator; // Really cant find a better name ATM

    // Get a tab and a toggle so that we don't have to re-deploy or power-cycle our bot to tune our arm setpoints
    private ShuffleboardTab tab = Shuffleboard.getTab("Optimized Arm");
    private GenericEntry tuningModeToggle = tab.add("Tuning Mode", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private boolean isTesting = !DriverStation.isFMSAttached(); // So we only run this if we are not at comp

    private Notifier updateTuningMode = new Notifier(() -> {

        // If we are driving, we want the arm to always be in brake mode, so we override it
        if(DriverStation.isEnabled()) {

            // Make sure the UI is updated
            tuningModeToggle.setBoolean(false);

            // Make sure we are in the correct mode
            setBrakeMode(true);

            // We don't care any further so skip
            return;
        }

        // Hopefully this reduces can load, but not sure
        if(isInBrakeMode != !tuningModeToggle.getBoolean(false)) { 

            // We negate so that it makes more sense in DS
            setBrakeMode(!tuningModeToggle.getBoolean(false));
        }
    });



    public OptimizedArm (boolean isTesting) {
        setupMotors();
        setBrakeMode(true);

        if(isTesting) {
            updateTuningMode.startPeriodic(0.5); // Slow as not to overwhelm our cpu
        }

        tuningModeToggle.setBoolean(false);

        graphStator = new ArmState(this);

        if(isTesting) {
            // Slow us tf down
            configMotionMagic(true, Constants.ArmConstants.BASE_MAX_V / 20, Constants.ArmConstants.BASE_MAX_A / 20, Constants.ArmConstants.BASE_CURVE_STR);
            configMotionMagic(false, Constants.ArmConstants.WRIST_MAX_V / 20, Constants.ArmConstants.WRIST_MAX_A / 20, Constants.ArmConstants.WRIST_CURVE_STR);

            // Limit our current
            bicepTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 20, 0.1));
            bicepTalonFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 20, 0.1));
            wristTalon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 20, 0.1));
        }
    }

    public void setupMotors() {
        bicepTalon = new WPI_TalonFX(Constants.ArmConstants.BASE_ID);
        bicepTalonFollower = new WPI_TalonFX(Constants.ArmConstants.BASE_FOLLOWER_ID);
        wristTalon = new WPI_TalonFX(Constants.ArmConstants.WRIST_ID);

        bicepTalon.setSelectedSensorPosition(0);

        bicepTalon.config_kF(0, Constants.ArmConstants.BASE_kF);
        bicepTalon.config_kP(0, Constants.ArmConstants.BASE_kP);
        bicepTalon.config_kI(0, Constants.ArmConstants.BASE_kI);
        bicepTalon.config_kD(0, Constants.ArmConstants.BASE_kD);

        configMotionMagic(true, Constants.ArmConstants.BASE_MAX_V, Constants.ArmConstants.BASE_MAX_A, Constants.ArmConstants.BASE_CURVE_STR);

        bicepTalonFollower.follow(bicepTalon);
        bicepTalonFollower.setInverted(InvertType.FollowMaster);

        wristTalon.setSelectedSensorPosition(0);
        
        wristTalon.config_kF(0, Constants.ArmConstants.WRIST_kF);
        wristTalon.config_kP(0, Constants.ArmConstants.WRIST_kP);
        wristTalon.config_kI(0, Constants.ArmConstants.WRIST_kI);
        wristTalon.config_kD(0, Constants.ArmConstants.WRIST_kD);

        configMotionMagic(false, Constants.ArmConstants.WRIST_MAX_V, Constants.ArmConstants.WRIST_MAX_A, Constants.ArmConstants.WRIST_CURVE_STR);
        
        wristTalon.setInverted(TalonFXInvertType.Clockwise);
    }

    public void periodic () {
        SmartDashboard.putNumber("Bicep Encoder in Degrees", getBicepPositionDegrees());
        SmartDashboard.putNumber("Wrist Encoder in Degrees", getWristPositionDegrees());
    }

    public ArmState getGraphStator () {
        return graphStator;
    }

    public void setBicep(ControlMode mode, double setpoint) {
        bicepTalon.set(mode, setpoint);
    }

    public void setWrist(ControlMode mode, double setpoint) {
        wristTalon.set(mode, setpoint);
    }

    public void setBrakeMode (boolean areBrakesOn) {
        bicepTalon.setNeutralMode(areBrakesOn ? NeutralMode.Brake : NeutralMode.Coast);
        bicepTalonFollower.setNeutralMode(areBrakesOn ? NeutralMode.Brake : NeutralMode.Coast);
        wristTalon.setNeutralMode(areBrakesOn ? NeutralMode.Brake : NeutralMode.Coast);

        isInBrakeMode = areBrakesOn;
    }

    public double getBicepPosition() {
        return bicepTalon.getSelectedSensorPosition();
    }

    public double getWristPosition() {
        return wristTalon.getSelectedSensorPosition();
    }

    public double getBicepPositionDegrees() {
        return ticksToDegreesBicep(getBicepPosition());
    }

    public double getWristPositionDegrees() {
        return ticksToDegreesWrist(getWristPosition());
    }

    public boolean isAt(double basePos, double wristPos) {
        return bicepAtTarget(basePos) && wristAtTarget(wristPos);
    }

    public boolean bicepAtTarget(double target) {
        return (Math.abs(bicepTalon.getSelectedSensorPosition() - target) < Constants.ArmConstants.BASE_ERROR_THRESHOLD);
    }

    public boolean wristAtTarget(double target) {
        return (Math.abs(wristTalon.getSelectedSensorPosition() - target) < Constants.ArmConstants.BASE_ERROR_THRESHOLD);
    }

    public double degreesToTicksBicep(double degrees) {
        return degrees * Math.PI / 180 / (Math.PI / 1024 / Constants.ArmConstants.BASE_GEAR_RATIO);
    }

    public double degreesToTicksWrist (double degrees) {
        return degrees * Math.PI / 180 / (Math.PI / 1024 / Constants.ArmConstants.WRIST_GEAR_RATIO);
    }

        /**
     * Should be the inverse of "degreesToTicksBicep"
     * @return the bicep position in degrees
     */
    public double ticksToDegreesBicep (double ticks) {
        return ticks * (Math.PI / 1024 / Constants.ArmConstants.BASE_GEAR_RATIO) * 180 * Math.PI;
    }

        /**
     * Should be the inverse of "degreesToTicksWrist"
     * @return the wrist position in degrees
     */
    public double ticksToDegreesWrist (double ticks) {
        return ticks * (Math.PI / 1024 / Constants.ArmConstants.WRIST_GEAR_RATIO) * 180 * Math.PI;
    }

    public double getBicepVelocity () {
        return bicepTalon.getSelectedSensorVelocity();
    }

    public void configMotionMagic (boolean isBase, double cruiseVelocity, double acceleration, int sCurveStrength) {
        if(isBase) {
            bicepTalon.configMotionCruiseVelocity(cruiseVelocity);
            bicepTalon.configMotionAcceleration(acceleration);
            bicepTalon.configMotionSCurveStrength(sCurveStrength);
        } else {
            wristTalon.configMotionCruiseVelocity(cruiseVelocity);
            wristTalon.configMotionAcceleration(acceleration);
            wristTalon.configMotionSCurveStrength(sCurveStrength);
        }
    }

    public void resetMotionMagic() {
        configMotionMagic(true, Constants.ArmConstants.BASE_MAX_V, Constants.ArmConstants.BASE_MAX_A, Constants.ArmConstants.BASE_CURVE_STR);
        configMotionMagic(false, Constants.ArmConstants.WRIST_MAX_V, Constants.ArmConstants.WRIST_MAX_A, Constants.ArmConstants.WRIST_CURVE_STR);
    }
}
