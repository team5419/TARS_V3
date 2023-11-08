package frc.robot.subsystems.arm;

import org.opencv.ml.ANN_MLP;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.math.Conversions;
import frc.lib.util.COTSFalconSwerveConstants.driveGearRatios;
import frc.robot.Constants;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.OptimizedArm.MotionMagicConfig;

public enum GraphStator {
  /**
   * Arranged like this
   *    E       B
   *        A
   *    D     F C
   */
  // We're missing states
  NOSTATE(LegalState.ILLEGAL, new ArmState(0, 0), new ArmState(0, 0), 
  // new ArmWaypoints[][] {
  //       new ArmWaypoints[] {null}, // A to NOSTATE, can be null if you want to error
  //       new ArmWaypoints[] {null}, // A to A
  //       new ArmWaypoints[] {null}, // A to B
  //       new ArmWaypoints[] {null}, // A to C
  //       new ArmWaypoints[] {null}, // A to D
  //       new ArmWaypoints[] {null}, // A to E
  //       new ArmWaypoints[] {null}
  // }
  null),

  // new ArmWaypoints[][] {
  //       new ArmWaypoints[] {null},
  //       new ArmWaypoints[] {null},
  //       new ArmWaypoints[] {null},
  //       new ArmWaypoints[] {null},
  //       new ArmWaypoints[] {null},
  //       new ArmWaypoints[] {null},
  //       new ArmWaypoints[] {null},
  //   }
  A(LegalState.LEGAL,
    new ArmState(-150, -80),
    new ArmState(150, 80),
    // This is brute force, we'd like to dynamically calculate this in the future
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null}, // A to NOSTATE, can be null if you want to error
        new ArmWaypoints[] {}, // A to A
        new ArmWaypoints[] {ArmWaypoints.QUAD_B}, // A to B
        new ArmWaypoints[] {ArmWaypoints.QUAD_E}, // A to C
        new ArmWaypoints[] {ArmWaypoints.QUAD_D}, // A to D
        new ArmWaypoints[] {ArmWaypoints.QUAD_C}, // A to E
        new ArmWaypoints[] {ArmWaypoints.QUAD_F}
    }),
  B(LegalState.LEGAL,
    new ArmState(65, 80),
    new ArmState(150, 215),
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null}, // To to nostate,
        new ArmWaypoints[] {ArmWaypoints.QUAD_A}, // To a
        new ArmWaypoints[] {}, // b (self)
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_C}, // c
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_D}, // d
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_E}, // e
        new ArmWaypoints[] {ArmWaypoints.QUAD_B, ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_F} // f
    }),
  C(LegalState.LEGAL,
    new ArmState(70, -215),
    new ArmState(150, -80),
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_B},
        new ArmWaypoints[] {}, // SELF
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_D},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_E},
        new ArmWaypoints[] {ArmWaypoints.QUAD_C, ArmWaypoints.QUAD_F}
    }),
  D(LegalState.LEGAL,
  new ArmState(-150, -215),
  new ArmState(-70, -80),
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_B},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_C},
        new ArmWaypoints[] {},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_E},
        new ArmWaypoints[] {ArmWaypoints.QUAD_E, ArmWaypoints.QUAD_F}
    }),
  E(LegalState.LEGAL,
    new ArmState(-150, 80),
    new ArmState(-70, 215),
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_B},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_C},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_D},
        new ArmWaypoints[] {},
        new ArmWaypoints[] {ArmWaypoints.QUAD_E, ArmWaypoints.QUAD_F}
    }),
  F(LegalState.LEGAL,
    new ArmState(23, -190),
    new ArmState(70, -70),
    new ArmWaypoints[][] {
      new ArmWaypoints[] {null},
      new ArmWaypoints[] {ArmWaypoints.QUAD_F, ArmWaypoints.QUAD_A},
      new ArmWaypoints[] {ArmWaypoints.QUAD_F, ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_B},
      new ArmWaypoints[] {ArmWaypoints.QUAD_F, ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_C},
      new ArmWaypoints[] {ArmWaypoints.QUAD_F, ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_D},
      new ArmWaypoints[] {ArmWaypoints.QUAD_F, ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_E},
      new ArmWaypoints[] {} // To self
    });

  final ArmState min, max;
  ArmWaypoints[][] pathTo;
  final LegalState legalState;

  GraphStator(LegalState legalState, ArmState min, ArmState max, ArmWaypoints[][] intermediatePath) {
    this.min = min;
    this.max = max;
    pathTo = intermediatePath;
    this.legalState = legalState;
  }

  public boolean inSector(double bicep, double wrist) {
    // logical equivalence
    // if(this == NOSTATE) {
    //     return false;
    // } else {
    //    return bicep > min.bicep && bicep < max.bicep &&  wrist > min.wrist && wrist < max.wrist;
    // }
    return (this != NOSTATE) && (bicep > min.bicep && bicep <= max.bicep) && (wrist > min.wrist && wrist <= max.wrist);
  }

  public static GraphStator getSectorStateFromCoords(double bicep, double wrist) {
    for (GraphStator sector : GraphStator.values()) {
      if (sector.inSector(bicep, wrist)) {
        return sector;
      }
    }
    return GraphStator.NOSTATE;
  }

  public static GraphStator getSectorStateFromCoords(ArmState state) {
    return getSectorStateFromCoords(state.bicep, state.wrist);
  }

  public static boolean isInSameSector(double x1, double y1, double x2, double y2) {
    return getSectorStateFromCoords(x1, y1) == getSectorStateFromCoords(x2, y2);
  }
  public static boolean isInSameSector(double x1, double y1, ArmState target2) {
    return isInSameSector(x1, y1, target2.bicep, target2.wrist);
  }
  public static boolean isInSameSector(double x1, double y1, ArmTargets target2) {
    return isInSameSector(x1, y1, target2.bicepTarget, target2.wristTarget);
  }
  public static boolean isInSameSector(ArmState target1, double x2, double y2) {
    return isInSameSector(target1.bicep, target1.wrist, x2, y2);
  }
  public static boolean isInSameSector(ArmState target1, ArmState target2) {
    return isInSameSector(target1.bicep, target1.wrist, target2.bicep, target2.wrist);
  }

  public static ArmWaypoints[] tracePath(ArmState startWaypoint, ArmTargets targetWaypoint) {
    // System.out.println(getSectorStateFromCoords(OptimizedArm.ticksToDegreesBicep(targetWaypoint.bicepTarget), OptimizedArm.ticksToDegreesWrist(targetWaypoint.wristTarget)).legalState);

    GraphStator start = getSectorStateFromCoords(startWaypoint);
    GraphStator end = getSectorStateFromCoords(OptimizedArm.ticksToDegreesBicep(targetWaypoint.bicepTarget), OptimizedArm.ticksToDegreesWrist(targetWaypoint.wristTarget));


    return start.pathTo[end.ordinal()];
  }

  /**
   * Returns a new motion magic config for the single part that needs to slow down
   * @param start - the staring waypoint
   * @param end - the ending waypoint
   * @return - a single motion magic config, to be passed into OptimizedArm.configMotionMagic
   */
  public static MotionMagicConfig[] calculateNewMotionMagic(Waypoint start, Waypoint end, OptimizedArm arm, boolean isLastMove) {
    double bicepDiff = Conversions.degreesToFalcon(Math.abs(start.bicep - end.bicep), Constants.ArmConstants.BASE_GEAR_RATIO);
    double wristDiff = Conversions.degreesToFalcon(Math.abs(start.wrist - end.wrist), Constants.ArmConstants.WRIST_GEAR_RATIO);

    double slowDownEvenMore = arm.isTesting ? 0.1 : 0.4;
    
    double ratio = bicepDiff / wristDiff;

    if(ratio < 0.0001) { // Prevent incredibly small ratios
        return new MotionMagicConfig[] { arm.getBaseConfig(true), arm.getBaseConfig(false) }; // change nothing, but have to return something
    }
    
    if(ratio < 1) {
      return new MotionMagicConfig[] {
        new MotionMagicConfig(true, Constants.ArmConstants.BASE_MAX_V * ratio * slowDownEvenMore, Constants.ArmConstants.BASE_MAX_A, isLastMove ? Constants.ArmConstants.BASE_CURVE_STR : 0), 
        new MotionMagicConfig(false, Constants.ArmConstants.WRIST_MAX_V * slowDownEvenMore, Constants.ArmConstants.WRIST_MAX_A, isLastMove ? Constants.ArmConstants.WRIST_CURVE_STR : 0)
      };
    } else if (ratio > 1) {
      return new MotionMagicConfig[] {
        new MotionMagicConfig(false, Constants.ArmConstants.WRIST_MAX_V / ratio * slowDownEvenMore, Constants.ArmConstants.WRIST_MAX_A, isLastMove ? Constants.ArmConstants.WRIST_CURVE_STR : 0),
        new MotionMagicConfig(true, Constants.ArmConstants.BASE_MAX_V * slowDownEvenMore, Constants.ArmConstants.BASE_MAX_A, isLastMove ? Constants.ArmConstants.BASE_CURVE_STR : 0)
      };
    } else {
        return new MotionMagicConfig[] { arm.getBaseConfig(true), arm.getBaseConfig(false) }; // change nothing, but have to return something
    }

    
    /**
     * bicep --> 100
     * wrist --> 50
     * 
     * ratio --> 2
     * 
     * would mean divide bicep by 2 (slow it down to match)
     * 
     * ---------------------------------------------------------------
     * 
     * bicep --> 50
     * wrist --> 100
     * 
     * ratio --> 0.5
     * 
     * would mean multiply wrist by 0.5 (slow it down to match)
     */
  }
}