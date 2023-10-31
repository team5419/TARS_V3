package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.OptimizedArm.MotionMagicConfig;

public enum GraphStator {
  /**
   * Arranged like this
   *
   *  E     B
   *     A
   *  D     C
   */
  // We're missing states
  NOSTATE(LegalState.ILLEGAL, new ArmState(0, 0), new ArmState(0, 0), null),
  A(LegalState.LEGAL,
    new ArmState(-45, -270),
    new ArmState(45, 270),
    // This is brute force, we'd like to dynamically calculate this in the future
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null}, // A to NOSTATE, can be null if you want to error
        new ArmWaypoints[] {}, // A to A
        new ArmWaypoints[] {ArmWaypoints.QUAD_B}, // A to B
        new ArmWaypoints[] {ArmWaypoints.QUAD_E}, // A to C
        new ArmWaypoints[] {ArmWaypoints.QUAD_D}, // A to D
        new ArmWaypoints[] {ArmWaypoints.QUAD_C} // A to E
    }),
  B(LegalState.LEGAL,
    new ArmState(45, 70),
    new ArmState(150, 270),
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null}, // To to nostate,
        new ArmWaypoints[] {ArmWaypoints.QUAD_A}, // To a
        new ArmWaypoints[] {}, // b (self)
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_C}, // c
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_D}, // d
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_E} // e
    }),
  C(LegalState.LEGAL,
    new ArmState(-45, 70),
    new ArmState(-150, 270),
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_B},
        new ArmWaypoints[] {}, // SELF
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_D},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_E}
    }),
  D(LegalState.LEGAL,
    new ArmState(-45, -70),
    new ArmState(150, -270),
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_B},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_C},
        new ArmWaypoints[] {},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_E}
    }),
  E(LegalState.LEGAL,
    new ArmState(45, -70),
    new ArmState(150, -270),
    new ArmWaypoints[][] {
        new ArmWaypoints[] {null},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_B},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_C},
        new ArmWaypoints[] {ArmWaypoints.QUAD_A, ArmWaypoints.QUAD_D},
        new ArmWaypoints[] {}
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
    return this != NOSTATE && bicep > min.bicep && bicep <= max.bicep && wrist > min.wrist
        && wrist <= max.wrist;
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
    return getSectorStateFromCoords(startWaypoint)
        .pathTo[getSectorStateFromCoords(targetWaypoint.bicepTarget, targetWaypoint.wristTarget).ordinal()];
  }

//   public static ArmWaypoints getFirstWaypoint(ArmSectors start, ArmState target) {
//     // switch (start) {
//     //   case SectorState.A:
//     // }
//     if ((start == A && getSectorStateFromCoords(target) == B) || (start == B && getSectorStateFromCoords(target) == A)) {
//       return ArmWaypoints.QUAD_B;
//     } else if ((start == A && getSectorStateFromCoords(target) == C) || (start == C && getSectorStateFromCoords(target) == A)) {
//       return ArmWaypoints.QUAD_C;
//     } else if ((start == A && getSectorStateFromCoords(target) == D) || (start == D && getSectorStateFromCoords(target) == A)) {
//       return ArmWaypoints.QUAD_D;
//     } else { // To and fro with E
//       return ArmWaypoints.QUAD_E;
//     }
//   }

  /**
   * Returns a new motion magic config for the single part that needs to slow down
   * @param start - the staring waypoint
   * @param end - the ending waypoint
   * @return - a single motion magic config, to be passed into OptimizedArm.configMotionMagic
   */
  public static MotionMagicConfig calculateNewMotionMagic(Waypoint start, Waypoint end, OptimizedArm arm) {
    double bicepDiff = Math.abs(start.bicep - end.bicep);
    double wristDiff = Math.abs(start.wrist - end.wrist);
    
    double ratio = bicepDiff / wristDiff;

    if(ratio < 0.0001) { // Prevent incredibly small ratios
        return arm.getBaseConfig(true); // change nothing, but have to return something
    }

    if(ratio > 1) {
        MotionMagicConfig base = arm.getBaseConfig(true);
        return new MotionMagicConfig(true, base.cruiseVelocity / ratio, base.acceleration, base.sCurveStrength);
    } else if (ratio < 1) {
        MotionMagicConfig base = arm.getBaseConfig(false);
        return new MotionMagicConfig(false, base.cruiseVelocity * ratio, base.acceleration, base.sCurveStrength);
    } else {
        return arm.getBaseConfig(true); // change nothing, but have to return something
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