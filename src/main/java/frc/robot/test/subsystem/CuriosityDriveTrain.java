package frc.robot.test.subsystem;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.base.input.Button;
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.StandardDriveTrain;
import frc.robot.base.device.motor.PhoenixMotorPair;
import frc.robot.base.device.motor.EncoderMotorConfig;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class CuriosityDriveTrain extends StandardDriveTrain {

    private Controller controller;
    private ADIS16448_IMU gyro = new ADIS16448_IMU();

    public static final EncoderMotorConfig CONFIG = new EncoderMotorConfig(
            3f/12f,
            360 * 4,
            0.92,
            0.8,
            0.0012,
            0.01,
            150
    );

    @Override
    public void control() {
        super.standardControl(controller);

        if(controller.buttonPressed(Button.B)) {
            this.resetDistance();
        }
    }

    public CuriosityDriveTrain(Controller controller) {
        super(
                new PhoenixMotorPair(
                        new TalonSRX(14),
                        new TalonSRX(13),
                        CONFIG
                ),
                new PhoenixMotorPair(
                        new TalonSRX(10),
                        new TalonSRX(12),
                        CONFIG
                ).invert(),
                5, 10, 10
        );
    }

    @Override
    public Map<String, Supplier<Object>> NTSets() {
        var newMap = new HashMap<>(super.NTSets());
        newMap.put("gyro", gyro::getAngle);
        return newMap;
    }
}
