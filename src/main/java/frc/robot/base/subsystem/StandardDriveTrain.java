package frc.robot.base.subsystem;

import frc.robot.base.util.Util;
import frc.robot.base.Controls;
import frc.robot.base.device.motor.EncoderMotor;
import frc.robot.base.device.motor.EncoderMotorConfig;
import frc.robot.base.input.Axis;
import frc.robot.base.input.Controller;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

/**
 * A drive train with two encoder motors and a rate limiter for each motor that is controlled with a controller
 * It features two driving modes, closed loop and open loop in case something goes wrong
 * NOTE: rate limiters are currently disabled
 */
public class StandardDriveTrain extends Subsystem {

    private EncoderMotor leftMotor; // 1.565
    private EncoderMotor rightMotor; // 1.565
    //private RateLimiter leftRateLimiter;
    //private RateLimiter rightRateLimiter;

    private double absoluteMaxSpeed;
    private double currentMaxSpeed;
    private double controllerDeadBand = 0.2;
    private int controllerPower = 2;

    private boolean useClosedLoop = true;
    private boolean reverseControl = false;

    public StandardDriveTrain(
            EncoderMotor leftMotor, EncoderMotor rightMotor,
            double maxAcceleration, double maxSpeed, double startMaxSpeed) {
        super("driveTrain");
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        //this.rightRateLimiter = new RateLimiter(maxAcceleration / 50);
        //this.leftRateLimiter = new RateLimiter(maxAcceleration / 50);
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

    private double maxScalingShift = 0;

    public void setMaxScaleShift(double shift) {
        this.maxScalingShift = shift;
    }

    public void setLeftVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setLeftVelocity(percent * (getCurrentMaxSpeed() + maxScalingShift));
        } else {
            this.setLeftPercentOutput(percent);
        }
    }

    public void setRightVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setRightVelocity(percent * (getCurrentMaxSpeed() + maxScalingShift));
        } else {
            this.setRightPercentOutput(percent);
        }
    }

    public void setVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setVelocity(percent * (getCurrentMaxSpeed() + maxScalingShift));
        } else {
            this.setPercentOutput(percent);
        }
    }

    public void standardControl(Controller controller) {
        int r = this.reverseControl ? -1 : 1;
        double fb = - r * Util.adjustInput(controller.getAxis(Controls.DriveTrain.DRIVE_FORWARD_BACKWARD), controllerDeadBand, controllerPower);
        double lr = r * Util.adjustInput(controller.getAxis(Controls.DriveTrain.TURN_LEFT_RIGHT), controllerDeadBand, controllerPower);

        double left = fb - lr;
        double right = fb + lr;
        
        if (useClosedLoop) {
            setLeftVelocity(left * absoluteMaxSpeed);
            setRightVelocity(right * absoluteMaxSpeed);
        } else {
            setLeftPercentOutput(left);
            setRightPercentOutput(right);
        }

        if (controller.buttonPressed(Controls.DriveTrain.USE_CLOSED_LOOP)) {
            this.useClosedLoop = true;
        }

        if (controller.buttonPressed(Controls.DriveTrain.DONT_USE_CLOSED_LOOP)) {
            this.useClosedLoop = false;
        }

        if (controller.buttonPressed(Controls.DriveTrain.TOGGLE_REVERSE)) {
            this.toggleReversed();
        }
        
        

        if(controller.getAxis(Axis.LEFT_TRIGGER) > 0.5) {
            this.resetDistance();
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
        //this.leftMotor.resetDistance();
        //this.rightMotor.resetDistance();
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

    public void setControllerDeadBand(double controllerDeadBand) {
        this.controllerDeadBand = controllerDeadBand;
    }

    public void setControllerPower(int controllerPower) {
        this.controllerPower = controllerPower;
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
    
    public void setReversed(boolean reverse) {
        this.reverseControl = reverse;
    }

    public void toggleReversed() {
        this.reverseControl = !this.reverseControl;
    }
    
    private Trajectory trajectory = new Trajectory();
    private long pathStartTime = 0;

    double ftPerMeter = 3.28084;
    
    public void followPath() {
        var state = trajectory.sample(System.currentTimeMillis() - pathStartTime);
        // this math is taken from looking at how they do it in the RamseteCommand class
        // around line 140 they do stuff with toWheelSpeeds which is where the math is
        var accel = state.accelerationMetersPerSecondSq;
        var curve = state.curvatureRadPerMeter * accel;
        /* "The track width of the drivetrain. Theoretically, this is the distance
   *     between the left wheels and right wheels. However, the empirical value may be larger than
   *     the physical measured value due to scrubbing effects." taken from DifferentialDriveKinematics */
        // TODO: set this value correctly (keeping units in mind)
        var trackWidthMeters = 1;
        var leftSpeed = accel - trackWidthMeters / 2 * curve;
        var rightSpeed = accel + trackWidthMeters / 2 * curve;
        
        setLeftVelocity(leftSpeed * ftPerMeter);
        setRightVelocity(rightSpeed * ftPerMeter);
        
    }
    
    public boolean finishedPath() {
        return System.currentTimeMillis() - pathStartTime >= trajectory.getTotalTimeSeconds() * 1000;
    }
    
    public void startTrajectory(Trajectory t) {
        this.trajectory = t;
        this.pathStartTime = System.currentTimeMillis();
    }
}
