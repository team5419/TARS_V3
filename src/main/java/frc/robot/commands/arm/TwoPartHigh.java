package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.abs;

// designed to move the arm higher up then down to avoid smacking the middle node
// internal logic is mostly the same
public class TwoPartHigh extends CommandBase {

    Arm mArm;

    boolean reachedPrep = false;

    public TwoPartHigh(Arm arm) {
        mArm = arm;

        addRequirements(mArm);
    }

    @Override
    public void initialize() {
        reachedPrep = false;
    }

    @Override
    public void execute() {
        // if the arm has not reached position one go there
        if (!reachedPrep) {
            mArm.baseTalonTarget = HIGH_BASE_POS_ALT_PREP;
            mArm.wristTalonTarget = HIGH_WRIST_POS_ALT - mArm.getHighWristFudge();

            if (mArm.armAtTarget() || mArm.safeForDouble()) {
                mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
            } else if (mArm.wristStowed()) {
                mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
            } else {
                mArm.stowWrist();
            }
        // once the arm is at position one start going to position two
        } else {
            if (mArm.getWristPos() < MID_WRIST_POS) {
                mArm.passSetpoints(HIGH_BASE_POS_ALT, HIGH_WRIST_POS_ALT - mArm.getHighWristFudge());
                System.out.println("Trying to end");
            } else {
                mArm.passSetpoints(HIGH_BASE_POS_ALT_PREP, HIGH_WRIST_POS_ALT - mArm.getHighWristFudge());
            }
        }

        // once the arm has reached position one slow down the arm and start going to position two
        if (!reachedPrep && abs(mArm.getBasePos() - HIGH_BASE_POS_ALT_PREP) <= BASE_ERROR_THRESHOLD/4) {
            reachedPrep = true;
            // prevents arm from going too far and hitting the riser
            mArm.setBaseBrake();
            mArm.configBaseAccel(BASE_MAX_A/12);
        }
    }

    // end once arm is at position
    @Override
    public boolean isFinished() {

        System.out.println("First: " + (abs(mArm.getWristPos() - (HIGH_WRIST_POS_ALT - mArm.getHighWristFudge())) < 1000));
        System.out.println("Second: " + (abs(mArm.getBasePos() - (HIGH_BASE_POS_ALT - mArm.getHighBaseFudge())) < 1000));

        return abs(mArm.getWristPos() - (HIGH_WRIST_POS_ALT - mArm.getHighWristFudge())) < 1000 &&
                abs(mArm.getBasePos() - (HIGH_BASE_POS_ALT - mArm.getHighBaseFudge())) < 1000;
    }

    // turn the accel configs back to normal once the command ends
    @Override
    public void end(boolean isFinished) {
        mArm.configBaseAccel(BASE_MAX_A);
        mArm.setTalonTargets(mArm.getBasePos(), mArm.getWristPos());
    }

}
