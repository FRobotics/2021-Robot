package frc.robot.hailfire.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.base.subsystem.Subsystem;
import frc.robot.base.util.PosControl;
import frc.robot.base.util.Util;
import frc.robot.hailfire.Controls;
import frc.robot.hailfire.IDs;
import frc.robot.base.device.motor.PhoenixMotor;
import frc.robot.base.device.motor.EncoderMotor;
import frc.robot.base.device.motor.Motor;
import frc.robot.hailfire.MotorConfig;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Shooter extends Subsystem {

    private EncoderMotor leftMotor = new PhoenixMotor(new TalonSRX(IDs.Shooter.LEFT_MOTOR), MotorConfig.Shooter.CONFIG);
    private EncoderMotor rightMotor = new PhoenixMotor(new TalonSRX(IDs.Shooter.RIGHT_MOTOR), MotorConfig.Shooter.CONFIG).invert();

    private Motor pitchMotor = new PhoenixMotor(new TalonSRX(IDs.Shooter.PITCH_MOTOR));
    private Encoder pitchEncoder = new Encoder(3, 4);
    private Motor carousel = new PhoenixMotor(new TalonSRX(IDs.Shooter.CAROUSEL_MOTOR)).invert();
    private DigitalInput carouselSwitch = new DigitalInput(1);
    private boolean carouselHit = false;

    private Relay spike = new Relay(0);

    // configurable variables for tuning from dash
    public double leftF;
    public double leftP;
    public double leftI;
    public double leftD;

    public double rightF;
    public double rightP;
    public double rightI;
    public double rightD;

    public double leftSpeedDemand;
    public double rightSpeedDemand;

    private double carouselOutput = 0;
    private boolean autoCarousel = false;
    private boolean spinForShooter = true;
    
    private boolean oldUpdateFPID = false;
    private boolean updateFPID = false;

    public Shooter() {
        super("shooter");
    }
    
    @Override
    public void periodic() {
        super.periodic();
        // update the config if the boolean updateFPID changed
        if (oldUpdateFPID != updateFPID) {
            oldUpdateFPID = updateFPID;
            
            var lc = MotorConfig.Shooter.CONFIG;
            lc.F = leftF;
            lc.P = leftP;
            lc.I = leftI;
            lc.D = leftD;
            leftMotor.setConfig(lc);
            
            var rc = MotorConfig.Shooter.CONFIG;
            rc.F = rightF;
            rc.P = rightP;
            rc.I = rightI;
            rc.D = rightD;
            rightMotor.setConfig(rc);
        }
    }

    @Override
    public void stop() {
        leftMotor.setPercentOutput(0);
        rightMotor.setPercentOutput(0);
        carousel.setPercentOutput(0);
        shooterStartTime = System.currentTimeMillis();
        carouselOutput = 0;
        autoCarousel = false;
    }

    @Override
    public void control() {

        if (Controls.Shooter.SHOOT()) {
            shoot(true);
        } else {
            spinForShooter = false;

            shooterStartTime = System.currentTimeMillis();

            leftMotor.setPercentOutput(0);
            rightMotor.setPercentOutput(0);
        }

        // I am fully aware that carousel control is a boolean mess
        // I should probably implement a simpler priority system
        // Either way, it works :)

        // manual control
        if (Controls.Shooter.MANUAL_CAROUSEL_RIGHT()) {
            carouselOutput = 0.7;
            autoCarousel = false;
        } else if (Controls.Shooter.MANUAL_CAROUSEL_LEFT()) {
            carouselOutput = -0.7;
            autoCarousel = false;
        } else if (Controls.Shooter.AUTO_CAROUSEL_LEFT()) { // semi manual (go to limit switch)
            carouselOutput = -0.7;
            autoCarousel = true;
        } else if (Controls.Shooter.AUTO_CAROUSEL_RIGHT()) {
            carouselOutput = 0.7;
            autoCarousel = true;
        } else if (autoCarousel) {
            // edge on detection
            if (!carouselSwitch.get()) {
                if (!carouselHit) {
                    carouselHit = true;
                    carouselOutput = 0;
                    autoCarousel = false;
                }
            } else {
                carouselHit = false;
            }
        } else { // if nothing else
            if (!spinForShooter) {
                carouselOutput = 0;
            }
        }

        carousel.setPercentOutput(carouselOutput);

        // move carousel up/down

        if (Controls.Shooter.CAROUSEL_DOWN()) {
            autoPitch = false;
            pitchMotor.setPercentOutput(.125);
        } else if (Controls.Shooter.CAROUSEL_UP()) {
            autoPitch = false;
            pitchMotor.setPercentOutput(-0.5);
        } else {
            if (Controls.Shooter.TEST_PITCH()) {
                autoPitch = true;
                pitchPosControl.target = pitchTarget / 600;
                pitchPosControl.reset();
            }
            if (autoPitch) {
                pitchAim();
            } else {
                pitchMotor.setPercentOutput(0);
            }
        }

        // turn on lights

        if (Controls.Shooter.TOGGLE_LIGHTS()) {
            spike.set(spike.get() == Relay.Value.kForward ? Relay.Value.kOff : Relay.Value.kForward);
        }
    }
    
    private long shooterStartTime = 0;

    public void shoot(boolean controlled) {
        // spin up motors and then carousel to shoot
        double carouselSpeed = .7;
        if (System.currentTimeMillis() - shooterStartTime > 2000 && !autoCarousel) {
            if (controlled) {
                carouselOutput = carouselSpeed;
                spinForShooter = true;
            } else {
                carousel.setPercentOutput(carouselSpeed);
            }
        } else if (!controlled) {
            carousel.setPercentOutput(0);
        }

        leftMotor.setVelocity(leftSpeedDemand); // 2600
        rightMotor.setVelocity(rightSpeedDemand); // 3000button
    }
    
    boolean autoPitch = false;
    PosControl pitchPosControl = new PosControl(0, 0.2, 0.01, 0.1, 1);
    double pitchTarget = 0;

    public void pitchAim() {
        if(pitchPosControl.isFinished()) {
            autoPitch = false;
            return;
        }
        var output = pitchPosControl.getSpeed(pitchEncoder.getDistance()/600);
        if(output > 0) {
            output = Math.min(1, output * 5);
        }
        pitchMotor.setPercentOutput(-output);
    }

    @Override
    public Map<String, Supplier<Object>> NTSets() {
        return Map.ofEntries(
                Map.entry("leftPercent", leftMotor::getOutputPercent),
                Map.entry("leftVelocity", leftMotor::getVelocity),
                Map.entry("rightPercent", rightMotor::getOutputPercent),
                Map.entry("rightVelocity", rightMotor::getVelocity),

                Map.entry("pitchOutput", pitchMotor::getOutputPercent),
                Map.entry("carouselOutput", carousel::getOutputPercent),
                Map.entry("carouselSwitch", carouselSwitch::get),
                Map.entry("lights", () -> spike.get() == Relay.Value.kForward),
                
                Map.entry("leftDistance", leftMotor::getDistance),
                Map.entry("rightDistance", rightMotor::getDistance),

                Map.entry("pitchDistance", pitchEncoder::getDistance)
        );
    }

    @Override
    public Map<String, Consumer<Object>> NTGets() {
        return Map.ofEntries(
                Util.<Double>setter("leftF", d -> leftF = d),
                Util.<Double>setter("leftP", d -> leftP = d),
                Util.<Double>setter("leftI", d -> leftI = d),
                Util.<Double>setter("leftD", d -> leftD = d),

                Util.<Double>setter("rightF", d -> rightF = d),
                Util.<Double>setter("rightP", d -> rightP = d),
                Util.<Double>setter("rightI", d -> rightI = d),
                Util.<Double>setter("rightD", d -> rightD = d),

                Util.<Double>setter("leftSpeedDemand", d -> leftSpeedDemand = d),
                Util.<Double>setter("rightSpeedDemand", d -> rightSpeedDemand = d),
                Util.<Boolean>setter("updateFPID", b -> updateFPID = b),
                Util.<Double>setter("pitchTarget", p -> pitchTarget = p)
        );
    }
}