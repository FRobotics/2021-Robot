package frc.robot.base.device;
import edu.wpi.first.wpilibj.AnalogInput;

// this was written by Darius
public class Pixy {
    private boolean targetFound;
    private double targetX;
    // Below this voltage threshold the pixyCam is considered to have NOT found the target
    private static final double pixyVoltageThreshold = 0.2d;
    private AnalogInput positionInput;

    public Pixy(int channel) {
        targetFound = false;
        targetX = 0.d;
        positionInput = new AnalogInput(channel);
    }

    public double getTarget() {
        return targetX;
    }

    public boolean foundTarget() {
        return targetFound;
    }

    /*
     * Reads from the pixyCam anolog input
     * Returns 0.0 if no object is found
     * Returns the position of the target in pixels if it is found
     */
    public double read() {
        double voltage = positionInput.getAverageVoltage();
        if (voltage < pixyVoltageThreshold) {
            targetFound = false;
            return 0.0;
        }
        targetFound = true;
        targetX = 200*voltage/3.3 - 100; // convert voltage to screen pixels, 200 pixels on screen / 3.3volts = 200/3.3 pixels per volt
        return targetX;
    }
}
