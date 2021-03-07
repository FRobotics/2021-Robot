package frc.robot.hailfire.subsystem;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.subsystem.StandardDriveTrain;
import frc.robot.base.util.DriveUtil;
import frc.robot.base.util.PosControl;
import frc.robot.base.util.Util;
import frc.robot.hailfire.Controls;
import frc.robot.hailfire.IDs;
import frc.robot.base.device.motor.PhoenixMotorPair;
import frc.robot.base.device.DoubleSolenoid4150;
import frc.robot.base.device.Pixy;
import frc.robot.hailfire.MotorConfig;
import frc.robot.hailfire.Vision;

public class DriveTrain extends StandardDriveTrain {

    private boolean reverseControl = false;
    public final ADIS16448_IMU gyro = new ADIS16448_IMU();

    private static final double LOW_MAX_SPEED = 5.5;

    private DoubleSolenoid4150 evoShifter = new DoubleSolenoid4150(
            IDs.DriveTrain.LEFT_EVO_SHIFTER_FORWARD,
            IDs.DriveTrain.LEFT_EVO_SHIFTER_REVERSE
    );

    private final Pixy pixy = new Pixy(3);

    private boolean autoShift = false;
    private boolean autoAim = false;
    
    public final Trajectory STRAIGHT = Util.loadTrajectory("/home/lvuser/Trajectory/test01_straight.json");
    public final Trajectory TURN_LEFT = Util.loadTrajectory("/home/lvuser/Trajectory/test02_turnLeft.json");
    public final Trajectory TURN_RIGHT = Util.loadTrajectory("/home/lvuser/Trajectory/test03_turnRight.json");
    public final Trajectory BACK_TO_START = Util.loadTrajectory("/home/lvuser/Trajectory/test04_BackToStart.json");
    
    public static PhoenixMotorPair createMotor(int master, int follower) {
        var motor = new PhoenixMotorPair(
            new TalonSRX(master),
            new VictorSPX(follower),
            MotorConfig.DriveTrain.LOW_CONFIG
        );
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setRampTime(0.5);
        return motor;
    }

    public DriveTrain() {
        super(
                new PhoenixMotorPair(
                        new TalonSRX(IDs.DriveTrain.LEFT_MOTOR_MASTER),
                        new VictorSPX(IDs.DriveTrain.LEFT_MOTOR_FOLLOWER),
                        MotorConfig.DriveTrain.LOW_CONFIG
                ),
                new PhoenixMotorPair(
                        new TalonSRX(IDs.DriveTrain.RIGHT_MOTOR_MASTER),
                        new VictorSPX(IDs.DriveTrain.RIGHT_MOTOR_FOLLOWER),
                        MotorConfig.DriveTrain.LOW_CONFIG
                ).invert(),
                10, 19, LOW_MAX_SPEED);
    }
    
    private PosControl posControl;
    private double startAngle = 0;
    private double angleX = 0;

    @Override
    public void control() {

        double turnSpeed = 0.2;

        if (Controls.DriveTrain.TURN_RIGHT()){
            setLeftVelOrPercent(-turnSpeed);
            setRightVelOrPercent(turnSpeed);
            this.autoAim = false;
        } else if (Controls.DriveTrain.TURN_LEFT()){
            setLeftVelOrPercent(turnSpeed);
            setRightVelOrPercent(-turnSpeed);
            this.autoAim = false;
        } else {
            if (Controls.DriveTrain.AUTO_AIM()) {
                // TODO: these are complete guesses
                posControl = new PosControl(angleX, 0.1, 0.2, 0.5, 5);
                this.autoAim = true;
                startAngle = gyro.getAngle();
            }
            if (this.autoAim) {
                // TODO: I have no idea if these units are compatible or if the direction is correct lmao
                double calculatedSpeed = posControl.getSpeed(this.gyro.getAngle() - startAngle);
                this.setLeftVelOrPercent(-calculatedSpeed);
                this.setRightVelOrPercent(calculatedSpeed);
            } else {
                DriveUtil.standardDrive(this, Controls.drive, reverseControl);
                if (Controls.DriveTrain.TOGGLE_REVERSE()) {
                    this.toggleReversed();
                }
            }
        }

        // shift gears

        if(Controls.DriveTrain.LOW_GEAR()){
            shiftToLowGear();
            autoShift = false;
        }

        if (Controls.DriveTrain.HIGH_GEAR()) {
            shiftToHighGear();
            autoShift = false;
        }

        if(Controls.DriveTrain.AUTO_SHIFT()) {
            autoShift = true;
        }

        if(autoShift) {
            if (
                Math.abs(getAverageDemand()) > 5
                && Math.abs(getAverageVelocity()) > 5
            ) {
                shiftToHighGear();
            }

            if (
                Math.abs(getAverageDemand()) < 4.5
                && Math.abs(getAverageVelocity()) < 4.5
            ) {
                shiftToLowGear();
            }
        }
    }

    public void shiftToHighGear() {
        if(evoShifter.extend()) {
            setMotorConfigs(MotorConfig.DriveTrain.HIGH_CONFIG);
            setCurrentMaxSpeed(getAbsoluteMaxSpeed());
        }
    }

    public void shiftToLowGear() {
        if(evoShifter.retract()) {
            setMotorConfigs(MotorConfig.DriveTrain.LOW_CONFIG);
            setCurrentMaxSpeed(LOW_MAX_SPEED);
        }
    }

    PosControl aimPosControl = new PosControl(0, 1, 0.5, 0.2, 0.5);;

    public void autoAim() {
        /* 
         * TODO: either use this code or discard it;
         * this constantly updates the target and uses a watchdog for safety in the vision class
         * I don't think we'll need this though + we'd have to change vision to work this way again
         */
        if(!Vision.isStale()) {
            double calculatedSpeed = aimPosControl.getSpeed(Vision.getYawOffset());
            this.setLeftVelOrPercent(-calculatedSpeed);
            this.setRightVelOrPercent(calculatedSpeed);
        }
    }

    @Override
    public Map<String, Consumer<Object>> NTGets() {
        return Map.ofEntries(Util.<Double>setter("/vision/data/angleX", 
            a -> this.angleX = a
        ));
    }
    
    @Override
    public Map<String, Supplier<Object>> NTSets() {
        Map<String, Supplier<Object>> sets = new HashMap<>();
        sets.putAll(super.NTSets());
        sets.putAll(Map.of("pixyReading", pixy::read));
        return sets;
    }

    public void setReversed(boolean reverse) {
        this.reverseControl = reverse;
    }

    public void toggleReversed() {
        this.reverseControl = !this.reverseControl;
    }
}
