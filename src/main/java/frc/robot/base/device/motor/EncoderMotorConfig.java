package frc.robot.base.device.motor;

public class EncoderMotorConfig {

    /**
     * Constructs a new motor config where distance is in length
     * @param wheelRadius the radius of the wheels in inches
     * @param countsPerRevolution how many counts the encoder gives per revolution
     * @param f ???
     * @param p ???
     * @param i ???
     * @param d ???
     * @param integralZone ???
     */
    public EncoderMotorConfig(double wheelRadius, int countsPerRevolution, double f, double p, double i, double d, int integralZone) {
        PID_LOOP_INDEX = 0;
        TIMEOUT_MS = 30;

        F = f;
        P = p;
        I = i;
        D = d;
        INTEGRAL_ZONE = integralZone;

        double wheel_circumference = wheelRadius * 2 * Math.PI;
        DISTANCE_MULTIPLIER = wheel_circumference / countsPerRevolution;
        // 10 * turns 100ms -> 1s
        INPUT_MULTIPLIER = 10 * DISTANCE_MULTIPLIER;
        OUTPUT_MULTIPLIER = 1 / INPUT_MULTIPLIER;
    }

    /**
     * Constructs a new config where distance is in revolutions
     * @param countsPerRevolution how many counts the encoder gives per revolution
     * @param f ???
     * @param p ???
     * @param i ???
     * @param d ???
     * @param integralZone ???
     */
    public EncoderMotorConfig(int countsPerRevolution, double f, double p, double i, double d, int integralZone) {
        PID_LOOP_INDEX = 0;
        TIMEOUT_MS = 30;

        F = f;
        P = p;
        I = i;
        D = d;
        INTEGRAL_ZONE = integralZone;

        DISTANCE_MULTIPLIER = 1d / countsPerRevolution;
        // 10 * turns 100ms -> 1s
        INPUT_MULTIPLIER = 10d * DISTANCE_MULTIPLIER * 60d;
        OUTPUT_MULTIPLIER = 1d / INPUT_MULTIPLIER;
    }

    public final int PID_LOOP_INDEX;
    public final int TIMEOUT_MS;
    public double F;
    public double P;
    public double I;
    public double D;
    public int INTEGRAL_ZONE;

    public final double DISTANCE_MULTIPLIER;
    public final double INPUT_MULTIPLIER;
    public final double OUTPUT_MULTIPLIER;
}
