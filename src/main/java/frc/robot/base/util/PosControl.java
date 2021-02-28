package frc.robot.base.util;

public class PosControl {

    /**
     * the distance the thing is trying to reach
     */
    public double target;


    /**
     * the distance before the thing reaches the target to stop to account for overshoot
     */
    private double deadBand;
    /**
     * how many times it should be on target before it finished in case of overshoot
     */
    public static final int HIT_COUNT_NEEDED = 3;
    /**
     * how many times the thing is on target in case of overshoot
     */
    private int hitCount;


    /**
     * when the thing should start to slow down
     */
    private double slowStart;
    /*
     * these adjust the output to create a linear equation (y = mx+b) that uses x values
     * from slowStart to deadBand (distance) and outputs y values from minSpeed to maxSpeed (speed output)
     */
    /**
     * multiplier for the output when slowing down
     */
    private double slowRate;
    /**
     * constant to add to the output when slowing down
     */
    private double slowOffset;


    /**
     * the top speed the thing can go
     */
    private double maxSpeed;


    public PosControl(double target, double slowRate, double deadBand, double minSpeed, double maxSpeed) {
        this.target = target;
        // this takes deadBand and adds the x distance of a line of slop slowRate that goes from minSpeed to maxSpeed
        this.slowStart = deadBand + (maxSpeed - minSpeed) / slowRate;
        // this divides by the distance it will go to create a range of 0 to 1 and then multiplies by the range needed (maxSpeed - minSpeed)
        this.slowRate = (maxSpeed - minSpeed) / (slowStart - deadBand);
        // this adds the minSpeed and FOILs out the deadBand part of (distance - deadBand) * slowRate to save time (do it beforehand)
        this.slowOffset = minSpeed - deadBand * slowRate;
        this.deadBand = deadBand;
        this.maxSpeed = maxSpeed;
    }

    public double getSpeed(double distanceTravelled) {
        // the difference in distance between where we are and where we want to be
        double diff = target - distanceTravelled;
        // the same thing but positive so we can do math without checking for negatives
        double absDiff = Math.abs(diff);

        if (absDiff < deadBand) {
            // if we're closer than the deadBand to the target then stop and add to the hitCounter
            hitCount++;
            return 0;
        }

        double output;

        if (absDiff < slowStart) {
            // if we're close enough to start slowing down then slow down
            // look at where slowRate and slowOffset are defined to see how this works
            output = absDiff * slowRate + slowOffset;
        } else {
            // otherwise go at full speed
            output = maxSpeed;
        }

        // this will be called if we did not stop which means we aren't there yet or overshot
        hitCount = 0;
        // this accounts for going backwards and overshooting (make it negative if it makes sense)
        return diff > 0 ? output : -output;
    }

    public boolean isFinished() {
        return hitCount >= HIT_COUNT_NEEDED;
    }

    public void reset() {
        this.hitCount = 0;
    }

}
