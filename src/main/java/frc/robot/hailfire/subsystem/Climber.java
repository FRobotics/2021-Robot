package frc.robot.hailfire.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.base.device.motor.Motor;
import frc.robot.base.device.motor.PhoenixMotor;
import frc.robot.base.util.Util;
import frc.robot.hailfire.Controls;
import frc.robot.hailfire.IDs;
import frc.robot.base.device.DoubleSolenoid4150;
import frc.robot.base.subsystem.Subsystem;

import java.util.Map;
import java.util.function.Supplier;

public class Climber extends Subsystem {

    private DoubleSolenoid4150 bottomSolenoid = new DoubleSolenoid4150(IDs.Climber.BOTTOM_SOLENOID_FORWARD, IDs.Climber.BOTTOM_SOLENOID_REVERSE);
    private Motor winch = new PhoenixMotor(new TalonSRX(IDs.Climber.WINCH_MOTOR));

    public Climber() {
        super("climber");
    }

    @Override
    public void control() {
        if(Controls.Climber.RETRACT()) {
            bottomSolenoid.retract();
        } else if(Controls.Climber.EXTEND()) {
            bottomSolenoid.extend();
        }

        if(Controls.Climber.SPIN_WINCH()) {
            winch.setPercentOutput(-0.9);
        }
        else {
            winch.setPercentOutput(0);
        }

        /*if(bottomSolenoid.isExtended()) {
            if(controller.buttonDown(Button.RIGHT_BUMPER)) {
                winch.setPercentOutput(0.9);
                System.out.println("test");
            }
        }*/
    }

    @Override
    public Map<String, Supplier<Object>> NTSets() {
        return Map.of(
            "bottomSolenoid", Util.solenoidNTV(bottomSolenoid)
        );
    }
}
