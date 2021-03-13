package frc.robot.base.subsystem;

import frc.robot.base.device.motor.EncoderMotor;
import frc.robot.base.device.motor.EncoderMotorConfig;

import java.util.Map;
import java.util.function.Supplier;

/**
 * A drive train with two encoder motors and a rate limiter for each motor that is controlled with a controller
 * It features two driving modes, closed loop and open loop in case something goes wrong
 * NOTE: rate limiters are currently disabled
 */
public class StandardDriveTrain extends Subsystem {

    private EncoderMotor leftMotor;
    private EncoderMotor rightMotor;
    private double absoluteMaxSpeed;
    private double currentMaxSpeed;

    private boolean useClosedLoop = true;

    public StandardDriveTrain(
            EncoderMotor leftMotor, EncoderMotor rightMotor,
            double maxAcceleration, double maxSpeed, double startMaxSpeed) {
        super("driveTrain");
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.currentMaxSpeed = startMaxSpeed;
        this.absoluteMaxSpeed = maxSpeed;
    }

    private double leftDemand = 0;
    private double leftOutputRaw = 0;

    private double rightDemand = 0;
    private double rightOutputRaw = 0;

    public void setLeftVelocity(double velocity) {
        velocity = safeVelocity(velocity);
        this.leftDemand = velocity;
        this.leftOutputRaw = this.leftMotor.setVelocity(velocity);
    }

    public void setRightVelocity(double velocity) {
        velocity = safeVelocity(velocity);
        this.rightDemand = velocity;
        this.rightOutputRaw = this.rightMotor.setVelocity(velocity);
    }

    public void setVelocity(double velocity) {
        setLeftVelocity(velocity);
        setRightVelocity(velocity);
    }

    public void setLeftPercentOutput(double percent) {
        percent = safePercent(percent);
        this.leftDemand = percent;
        this.leftOutputRaw = percent;
        this.leftMotor.setPercentOutput(percent);
    }

    public void setRightPercentOutput(double percent) {
        percent = safePercent(percent);
        this.rightDemand = percent;
        this.rightOutputRaw = percent;
        this.rightMotor.setPercentOutput(percent);
    }

    public void setPercentOutput(double percent) {
        this.setLeftPercentOutput(percent);
        this.setRightPercentOutput(percent);
    }

    public void setLeftVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setLeftVelocity(percent * getCurrentMaxSpeed());
        } else {
            this.setLeftPercentOutput(percent);
        }
    }

    public void setRightVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setRightVelocity(percent * getCurrentMaxSpeed());
        } else {
            this.setRightPercentOutput(percent);
        }
    }

    public void setVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setVelocity(percent * getCurrentMaxSpeed());
        } else {
            this.setPercentOutput(percent);
        }
    }

    public double safeVelocity(double velocity) {
        return Math.max(Math.min(velocity, currentMaxSpeed), -currentMaxSpeed);
    }

    public double safePercent(double percent) {
        return Math.max(Math.min(percent, 1), -1);
    }

    @Override
    public void stop() {
        setPercentOutput(0);
        this.leftMotor.resetDistance();
        this.rightMotor.resetDistance();
    }

    @Override
    public Map<String, Supplier<Object>> NTSets() {
        return Map.ofEntries(
                Map.entry("left/velocity", leftMotor::getVelocity),
                Map.entry("left/distance", leftMotor::getDistance),
                Map.entry("left/demand", () -> leftDemand),
                Map.entry("left/demandRaw", () -> leftOutputRaw),
                Map.entry("left/received/velocity", leftMotor::getVelocityRaw),
                Map.entry("left/received/outputPercent", leftMotor::getOutputPercent),

                Map.entry("right/velocity", rightMotor::getVelocity),
                Map.entry("right/distance", rightMotor::getDistance),
                Map.entry("right/demand", () -> rightDemand),
                Map.entry("right/demandRaw", () -> rightOutputRaw),
                Map.entry("right/received/velocity", rightMotor::getVelocityRaw),
                Map.entry("right/received/outputPercent", rightMotor::getOutputPercent),

                Map.entry("closedLoopControl", () -> useClosedLoop)
        );
    }

    public void setCurrentMaxSpeed(double maxSpeed) {
        this.currentMaxSpeed = maxSpeed;
    }

    public double getCurrentMaxSpeed() {
        return this.currentMaxSpeed;
    }

    public void setClosedLoop(boolean useClosedLoop) {
        this.useClosedLoop = useClosedLoop;
    }

    public boolean isClosedLoop() { // added 3/7 KW
        return this.useClosedLoop;
    }

    public double getLeftDemand() {
        return this.leftDemand;
    }

    public double getRightDemand() {
        return this.rightDemand;
    }

    public double getAverageDemand() {
        return (this.leftDemand + this.rightDemand) / 2;
    }

    public double getLeftVelocity() {
        return this.leftMotor.getVelocity();
    }

    public double getRightVelocity() {
        return this.leftMotor.getVelocity();
    }

    public double getAverageVelocity() {
        return (this.leftMotor.getVelocity() + this.rightMotor.getVelocity()) / 2;
    }
    
    public double getLeftDistance() {
        return leftMotor.getDistance();
    }
    
    public double getRightDistance() {
        return rightMotor.getDistance();
    }

    public double getAverageDistance() {
        return (leftMotor.getDistance() + rightMotor.getDistance()) / 2;
    }

    public double getAbsoluteMaxSpeed() {
        return this.absoluteMaxSpeed;
    }
    public void setLeftMotorConfig(EncoderMotorConfig config) {
        this.leftMotor.setConfig(config);
    }

    public void setRightMotorConfig(EncoderMotorConfig config) {
        this.rightMotor.setConfig(config);
    }

    public void setMotorConfigs(EncoderMotorConfig config) {
        setLeftMotorConfig(config);
        setRightMotorConfig(config);
    }

    public void resetDistance() {
        this.leftMotor.resetDistance();
        this.rightMotor.resetDistance();
    }
}
