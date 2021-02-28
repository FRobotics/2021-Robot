package frc.robot.test.subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.base.subsystem.TestSubsystem;

import java.util.Map;
import java.util.function.Supplier;

public class LimitSwitchTest extends TestSubsystem {

    private DigitalInput limitSwitch = new DigitalInput(0);

    @Override
    public Map<String, Supplier<Object>> NTSets() {
        return Map.of(
            "limitSwitch", limitSwitch::get
        );
    }
}
