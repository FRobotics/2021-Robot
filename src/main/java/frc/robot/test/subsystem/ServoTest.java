package frc.robot.test.subsystem;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.base.input.Button;
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.TestSubsystem;

public class ServoTest extends TestSubsystem {

    private Controller controller;

    Servo servo = new Servo(0);

    public ServoTest(Controller controller) {
        this.controller = controller;
    }

    @Override
    public void stop() {
        servo.set(0);
    }

    @Override
    public void control() {
        if(controller.buttonDown(Button.X)) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0);
        }
    }
}
