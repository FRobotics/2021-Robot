package frc.robot.base.util;

/**
 * prevents a value from changing too much
 */
public class RateLimiter {

    private double maxChange;
    private double lastVal;

    public RateLimiter(double maxChange, double startVal) {
        this.maxChange = maxChange;
        this.lastVal = startVal;
    }

    public RateLimiter(double maxChange) {
        this(maxChange, 0);
    }

    public double get(double val) {
        double output;

        double diff = val - lastVal;
        if(Math.abs(diff) > maxChange) {
            output = lastVal + (diff > 0 ? maxChange : -maxChange);
        } else {
            output = val;
        }

        lastVal = output;
        return output;
    }

    public void setMaxChange(double maxChange) {
        this.maxChange = maxChange;
    }
}
