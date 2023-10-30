package frc.robot.subsystems.arm;

import java.util.HashMap;

import frc.robot.Constants.ArmTargets;
import frc.robot.subsystems.arm.OptimizedArm.MotionMagicConfig;

public class GraphStator {
    class Sector {
        double bicepMin;
        double bicepMax;

        double wristMin;
        double wristMax;

        public Sector (double bicepMin, double bicepMax, double wristMin, double wristMax) {
            this.bicepMin = bicepMin;
            this.bicepMax = bicepMax;

            this.wristMin = wristMin;
            this.wristMax = wristMax;
        }
    }

    public enum SectorState {
        A, 
        B, 
        C, 
        D, 
        E,
        NOSTATE
    }

    HashMap<SectorState, Sector> sectorLookup = new HashMap<SectorState, Sector>();

    public static class Waypoint {
        public double bicep; // x
        public double wrist; // y

        public Waypoint(double bicep, double wrist) {
            this.bicep = bicep;
            this.wrist = wrist;
        }
    }

    OptimizedArm arm;

    /**
     * Arranged like this
     * 
     *  E     B
     *     A
     *  D     C
     */
    public GraphStator (OptimizedArm arm) {
        sectorLookup.put(SectorState.A, new Sector(-45, 45, -270, 270));
        sectorLookup.put(SectorState.B, new Sector(45, 150, 70, 270));
        sectorLookup.put(SectorState.C, new Sector(-45, -150, 70, 270));
        sectorLookup.put(SectorState.D, new Sector(-45, 150, -70, -270));
        sectorLookup.put(SectorState.E, new Sector(45, 150, -70, -270));

        this.arm = arm;
    }

    public SectorState getSectorStateFromCoords(double x, double y) {
        if(isInSector(x, y, sectorLookup.get(SectorState.A))) {
            return SectorState.A;
        } else if (isInSector(x, y, sectorLookup.get(SectorState.B))) {
            return SectorState.B;
        } else if (isInSector(x, y, sectorLookup.get(SectorState.C))) {
            return SectorState.C;
        } else if (isInSector(x, y, sectorLookup.get(SectorState.D))) {
            return SectorState.D;
        } else if (isInSector(x, y, sectorLookup.get(SectorState.E))) {
            return SectorState.E;
        } else {
            return SectorState.NOSTATE;
        }
    }

    public SectorState getSectorStateFromCoords(Waypoint waypoint) {
        if(isInSector(waypoint.bicep, waypoint.wrist, sectorLookup.get(SectorState.A))) {
            return SectorState.A;
        } else if (isInSector(waypoint.bicep, waypoint.wrist, sectorLookup.get(SectorState.B))) {
            return SectorState.B;
        } else if (isInSector(waypoint.bicep, waypoint.wrist, sectorLookup.get(SectorState.C))) {
            return SectorState.C;
        } else if (isInSector(waypoint.bicep, waypoint.wrist, sectorLookup.get(SectorState.D))) {
            return SectorState.D;
        } else if (isInSector(waypoint.bicep, waypoint.wrist, sectorLookup.get(SectorState.E))) {
            return SectorState.E;
        } else {
            return SectorState.NOSTATE;
        }
    }

    public boolean isInSector(double x, double y, Sector sector) {
        return (x < sector.bicepMax && x > sector.bicepMin) && (y < sector.wristMax && y > sector.wristMin);
    }

    public boolean isInSector(double x, double y, SectorState sector) {
        return isInSector(x, y, sectorLookup.get(sector));
    }

    public boolean isInSameSector(double x1, double y1, double x2, double y2) {
        return getSectorStateFromCoords(x1, y1) == getSectorStateFromCoords(x2, y2);
    }

    public boolean isInSameSector(double x1, double y1, ArmTargets target) {
        return getSectorStateFromCoords(x1, y1) == getSectorStateFromCoords(arm.ticksToDegreesBicep(target.bicepTarget), arm.ticksToDegreesWrist(target.wristTarget));
    }

    public Waypoint newWaypoint (double x, double y) {
        return new Waypoint(x, y);
    }


    public Waypoint getNextWaypoint(SectorState state, Waypoint target) {
        if((state == SectorState.A && getSectorStateFromCoords(target) == SectorState.B) || (state == SectorState.B && getSectorStateFromCoords(target) == SectorState.A)) {
            return new Waypoint(45, 70);
        } else if ((state == SectorState.A && getSectorStateFromCoords(target) == SectorState.C) || (state == SectorState.C && getSectorStateFromCoords(target) == SectorState.A)) {
            return new Waypoint(45, -70);
        } else if ((state == SectorState.A && getSectorStateFromCoords(target) == SectorState.D) || (state == SectorState.D && getSectorStateFromCoords(target) == SectorState.A)) {
            return new Waypoint(-45, -70);
        } else { // To and fro with E
            return new Waypoint(-45, 70);
        }
    }

    public Waypoint[] tracePath(Waypoint startWaypoint, Waypoint targetWaypoint) {
        if (getSectorStateFromCoords(startWaypoint) == SectorState.A) { // If we are in a, it can only be one point
            return new Waypoint[] { getNextWaypoint(SectorState.A, targetWaypoint) };

        } else if (getSectorStateFromCoords(targetWaypoint) == SectorState.A) { // If we are ending in a, in can also only be one point
            return new Waypoint[] { getNextWaypoint(getSectorStateFromCoords(startWaypoint), targetWaypoint) };

        } else { // If neither of those, that means we need two points to make our path
            return new Waypoint[] { 
                getNextWaypoint(getSectorStateFromCoords(startWaypoint), new Waypoint(0, 0)), // From wherever to a
                getNextWaypoint(getSectorStateFromCoords(new Waypoint(0, 0)), targetWaypoint) // From a to destination
            };
        }
    }

    /**
     * Returns a new motion magic config for the single part that needs to slow down
     * @param start - the staring waypoint
     * @param end - the ending waypoint
     * @return - a single motion magic config, to be passed into OptimizedArm.configMotionMagic
     */
    public MotionMagicConfig calculateNewMotionMagic(Waypoint start, Waypoint end) {
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
