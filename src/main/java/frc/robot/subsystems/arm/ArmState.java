package frc.robot.subsystems.arm;

import java.util.HashMap;

import frc.robot.Constants.ArmTargets;

public class ArmState {
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

    public class Waypoint {
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
     *  D     c
     */
    public ArmState (OptimizedArm arm) {
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

    public Waypoint[] tracePath(Waypoint currentWaypoint, Waypoint targetWaypoint) {
        Waypoint[] tempList = new Waypoint[2];

        if (getSectorStateFromCoords(currentWaypoint) == SectorState.A) { // If we are in a, it can only be one point
            return new Waypoint[] { getNextWaypoint(SectorState.A, targetWaypoint) };

        } else if (getSectorStateFromCoords(targetWaypoint) == SectorState.A) { // If we are ending in a, in can also only be one point
            return new Waypoint[] { getNextWaypoint(getSectorStateFromCoords(currentWaypoint), targetWaypoint) };

        } else { // If neither of those, that means we need two points to go to and fro
            tempList[0] = getNextWaypoint(getSectorStateFromCoords(currentWaypoint), new Waypoint(0, 0));
            tempList[1] = getNextWaypoint(getSectorStateFromCoords(tempList[0]), targetWaypoint);
            return tempList;
        }
    }
}
