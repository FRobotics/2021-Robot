package frc.robot.base.device.motor;

/**
 * A generic motor interface that lets you use generic motors in classes to make switching motors very easy
 */
public interface Motor {

    /**
     * Sets the percent output of the motor
     * @param percent the percent output (-1 to 1)
     */
    void setPercentOutput(double percent);
    double getOutputPercent();

    /**
     * Sets whether the output of the motor should be inverted or not
     * @param inverted whether the motor should be inverted or not
     * @return the motor for convenience
     */
    Motor setInverted(boolean inverted);

    /**
     * Switches whether the motor is inverted or not;
     * if it's inverted it will make it not, and if it's not it will make it inverted
     * @return the motor for convenience
     */
    Motor invert();
}