package frc.robot.hailfire;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.base.NTHandler;

public class Vision {

    // NOTE: old NTV names and variables, this will probably get deleted
    private static NetworkTableEntry watchdogEntry = NTHandler.getVisionEntry("watchdog");
    private static double lastWatchdogVal = 0;
    private static long staleStart = 0;

    private static NetworkTableEntry targetFoundEntry = NTHandler.getVisionEntry("targetFound");

    private static NetworkTableEntry yawEntry = NTHandler.getVisionEntry("yawOffset");
    private static NetworkTableEntry pitchEntry = NTHandler.getVisionEntry("pitchOffset");

    private static double yawOffset = 0;
    private static double pitchOffset = 0;

    public static double getYawOffset() {
        return isStale()
                ? yawOffset
                : (yawOffset = yawEntry.getDouble(0));
    }

    public static double getPitchOffset() {
        return isStale()
                ? pitchOffset
                : (pitchOffset = pitchEntry.getDouble(0));
    }

    public static boolean isStale() {
        return System.currentTimeMillis() - staleStart > 500;
    }

    public static void update() {
        double currentVal = watchdogEntry.getDouble(0);
        if(currentVal != lastWatchdogVal && targetFoundEntry.getBoolean(false)) {
            staleStart = System.currentTimeMillis();
        }
        lastWatchdogVal = currentVal;
    }

}
