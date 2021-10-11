package frc.robot.hailfire.subsystem;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;  //JAS not used

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Encoder;  //JAS not used
import frc.robot.base.util.Util;
import frc.robot.hailfire.Controls;
import frc.robot.hailfire.IDs;
import frc.robot.base.subsystem.Subsystem;
import frc.robot.base.device.motor.PhoenixMotor;
import frc.robot.base.device.DoubleSolenoid4150;
import frc.robot.base.device.motor.Motor;

import java.util.Map;
import java.util.function.Supplier;

public class Intake extends Subsystem {

    public DoubleSolenoid4150 solenoid = new DoubleSolenoid4150(IDs.Intake.ARM_FORWARD, IDs.Intake.ARM_REVERSE);
    public Motor spinner = new PhoenixMotor(new VictorSPX(IDs.Intake.MOTOR));
    //private Motor pitchMotor = new PhoenixMotor(new TalonSRX(IDs.Shooter.PITCH_MOTOR));
    //private Encoder pitchEncoder = new Encoder(3, 4);
    public DigitalInput sensor = new DigitalInput(IDs.Intake.SENSOR);

    //jas added for intake sequence
    public boolean allowIntakeSequence = false;
    //jas added for intake sequence
    public boolean ballDetectSensor = false;

    public Intake() {
        super("intake");
    }
    @Override
    public void auto() {
        ballDetectSensor = sensor.get();
        //JAS System.out.println("Good");
    }

    @Override
    public void stop() {
        //System.out.println("WHY");
        //spinner.setPercentOutput(0);
    }
    
    @Override
    public void control() {

        //jas added (maybe should be in periodic ??)
        ballDetectSensor = sensor.get();

        if (Controls.Intake.ARM_UP()) {
            solenoid.retract();
        }

        if (Controls.Intake.ARM_DOWN()) {
            solenoid.extend();
        }
        
        // spin if solenoid is out

        if (Controls.Intake.SPIN_FORWARD() && solenoid.isExtended()) {
            //if (pitchEncoder.getDistance() > 0) {
                //pitchMotor.setPercentOutput(.75);
            //} else {
            spinner.setPercentOutput(1);
            allowIntakeSequence = false;
            //}
        } else if(Controls.Intake.SPIN_BACKWARD() && solenoid.isExtended()) {
            spinner.setPercentOutput(-1);
            allowIntakeSequence = false;
        } else {
            //JAS added
            if ( !Controls.Intake.INTAKE_SEQ()) {
                spinner.setPercentOutput(0);
            }
            allowIntakeSequence = true;
        }
    }

    //JAS added
    public boolean getBallDetectSensor() {
        return ballDetectSensor;
    }

    //JAS added
    public void setSpinnerOutput( double value ) {
        spinner.setPercentOutput(value);
    }

    @Override
    public Map<String, Supplier<Object>> NTSets() {
        return Map.of(
            "solenoid", Util.solenoidNTV(solenoid),
            "motor", spinner::getOutputPercent,
            "sensor", this::getBallDetectSensor      //JAS changed.  Used to read I/O directly    
        );
    }

}