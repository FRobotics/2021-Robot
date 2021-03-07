package frc.robot.base.device.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

/**
 * A phoenix motor that implements the EncoderMotor interface to allow you to make drive systems generic
 */
public class PhoenixMotor implements EncoderMotor {

    private double inputMultiplier;
    private double outputMultiplier;
    private double distanceMultiplier;
    private BaseMotorController motor;

    public PhoenixMotor(BaseMotorController motor) {
        this(motor, null);
    }

    public PhoenixMotor(BaseMotorController motor, EncoderMotorConfig config) {
        this.motor = motor;
        this.motor.configFactoryDefault();
        this.motor.configNominalOutputForward(0);
        this.motor.configNominalOutputReverse(0);
        this.motor.configNeutralDeadband(.001);
        this.motor.setSensorPhase(false);
        if(config != null) {
            setConfig(config);
        } else {
            inputMultiplier = 1;
            outputMultiplier = 1;
            distanceMultiplier = 1;
        }
    }

    @Override
    public void setConfig(EncoderMotorConfig config) {
        int slotIdx = config.PID_LOOP_INDEX;
        int timeoutMS = config.TIMEOUT_MS;

        this.motor.config_kF(slotIdx, config.F, timeoutMS);
        this.motor.config_kP(slotIdx, config.P, timeoutMS);
        this.motor.config_kI(slotIdx, config.I, timeoutMS);
        this.motor.config_kD(slotIdx, config.D, timeoutMS);
        this.motor.config_IntegralZone(slotIdx, config.INTEGRAL_ZONE, timeoutMS);

        this.motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, slotIdx, timeoutMS);

        inputMultiplier = config.INPUT_MULTIPLIER;
        outputMultiplier = config.OUTPUT_MULTIPLIER;
        distanceMultiplier = config.DISTANCE_MULTIPLIER;
    }

    @Override
    public double setVelocity(double velocity) {
        double rawOutput = velocity * outputMultiplier;
        motor.set(ControlMode.Velocity, rawOutput);
        return rawOutput;
    }

    @Override
    public void setPercentOutput(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public double getOutputPercent() {
        return motor.getMotorOutputPercent();
    }

    @Override
    public PhoenixMotor setInverted(boolean inverted) {
        motor.setInverted(inverted);
        return this;
    }

    @Override
    public PhoenixMotor invert() {
        motor.setInverted(!motor.getInverted());
        return this;
    }

    /**
     * NOTE: this only works if it was created using config and has an encoder
     * @return the velocity of the motor in feet per second
     */
    @Override
    public double getVelocity() {
        return getVelocityRaw() * inputMultiplier;
    }

    /**
     * NOTE: this only works if it was created using config and has an encoder
     * @return the raw value of the velocity of the motor
     */
    @Override
    public double getVelocityRaw() {
        return motor.getSelectedSensorVelocity();
    }

    /**
     * NOTE: this only works if it was created using config and has an encoder
     * @return distance the motor has spun in feet
     */
    @Override
    public double getDistance() {
        return motor.getSelectedSensorPosition() * distanceMultiplier;
    }

    @Override
    public void resetDistance() {
        motor.setSelectedSensorPosition(0);
    }

    public void follow(PhoenixMotor motor) {
        this.motor.follow(motor.motor);
    }
    
    public void setNeutralMode(NeutralMode mode) {
        this.motor.setNeutralMode(mode);
    }
    
    public void setRampTime(double time) {
        this.motor.configOpenloopRamp(time);
        this.motor.configClosedloopRamp(time);
    }

}