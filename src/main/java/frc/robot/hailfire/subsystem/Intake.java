package frc.robot.hailfire.subsystem;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.base.util.Util;
import frc.robot.hailfire.Controls;
import frc.robot.hailfire.IDs;
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.Subsystem;
import frc.robot.base.device.motor.PhoenixMotor;
import frc.robot.base.device.DoubleSolenoid4150;
import frc.robot.base.device.motor.Motor;

import java.util.Map;
import java.util.function.Supplier;

public class Intake extends Subsystem {

    private Controller controller;

    private DoubleSolenoid4150 solenoid = new DoubleSolenoid4150(IDs.Intake.ARM_FORWARD, IDs.Intake.ARM_REVERSE);
    private Motor spinner = new PhoenixMotor(new VictorSPX(IDs.Intake.MOTOR));

    public Intake(Controller controller) {
        super("intake");
        this.controller = controller;
    }

    @Override
    public void stop() {
        spinner.setPercentOutput(0);
    }
    
    @Override
    public void control() {

        if (controller.buttonPressed(Controls.Intake.ARM_UP)) {
            solenoid.retract();
        }

        if (controller.buttonPressed(Controls.Intake.ARM_DOWN)) {
            solenoid.extend();
        }
        
        // spin if solenoid is out

        if (controller.buttonDown(Controls.Intake.SPIN_FORWARD) && solenoid.isExtended()) {
            spinner.setPercentOutput(1);
        } else if(controller.buttonDown(Controls.Intake.SPIN_BACKWARD) && solenoid.isExtended()) {
            spinner.setPercentOutput(-1);
        } else {
            spinner.setPercentOutput(0);
        }
    }

    @Override
    public Map<String, Supplier<Object>> NTSets() {
        return Map.of(
            "solenoid", Util.solenoidNTV(solenoid),
            "motor", spinner::getOutputPercent
        );
    }

}