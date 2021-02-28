package frc.robot.base.device.motor;

/**
 * A generic motor that has an encoder
 */
public interface EncoderMotor extends Motor {
    double getVelocity();
    double getVelocityRaw();
    double setVelocity(double speed);
    double getDistance();
    void resetDistance();
    @Override
    EncoderMotor setInverted(boolean inverted);
    @Override
    EncoderMotor invert();

    void setConfig(EncoderMotorConfig config);
}