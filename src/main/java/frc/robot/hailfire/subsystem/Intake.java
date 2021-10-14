package frc.robot.hailfire.subsystem;
//import frc.robot.base.NTHandler;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;  //JAS not used

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Encoder;  //JAS not used
import frc.robot.base.util.Util;
import frc.robot.hailfire.Controls;
import frc.robot.hailfire.Hailfire;
//import frc.robot.hailfire.Hailfire;
import frc.robot.hailfire.IDs;
import frc.robot.base.subsystem.Subsystem;
import frc.robot.base.device.motor.PhoenixMotor;
import frc.robot.base.device.DoubleSolenoid4150;
import frc.robot.base.device.motor.Motor;
import edu.wpi.first.networktables.NetworkTableEntry; //jas added

import java.util.Map;
import java.util.function.Supplier;

public class Intake extends Subsystem {

    public DoubleSolenoid4150 solenoid = new DoubleSolenoid4150(IDs.Intake.ARM_FORWARD, IDs.Intake.ARM_REVERSE);
    public Motor spinner = new PhoenixMotor(new VictorSPX(IDs.Intake.MOTOR));
    //private Motor pitchMotor = new PhoenixMotor(new TalonSRX(IDs.Shooter.PITCH_MOTOR));
    //private Encoder pitchEncoder = new Encoder(3, 4);
    public DigitalInput sensor = new DigitalInput(IDs.Intake.SENSOR);

    //jas added for intake sequence
    private boolean initIntakeSequence = true;
    private NetworkTableEntry intakeSeqCarPitchDmdNTEntry;
    private NetworkTableEntry intakeSeqCarTurnDmdNTEntry;
    private NetworkTableEntry intakeSeqIntakeSpinDmdNTEntry;
    private NetworkTableEntry intakeSeqStateNTEntry;
    private boolean internalIntakeSeqInProg = false;


    //jas added for intake sequence
    public boolean ballDetectSensor = false;

    public Intake() {
        super("intake");

        //jas added intake seq
        //intakeSeqCarPitchDmdNTEntry = NTHandler.getRobotEntry("IntakeSeq/CarPitchDmd");
        //intakeSeqCarTurnDmdNTEntry = NTHandler.getRobotEntry("IntakeSeq/CarTurnDmd");
        //intakeSeqIntakeSpinDmdNTEntry = NTHandler.getRobotEntry("IntakeSeq/IntakeSpinDmd");
        //intakeSeqStateNTEntry = NTHandler.getRobotEntry("IntakeSeq/State");
        //intakeSeqIntakeSpinDmdNTEntry.setDouble(intakeSeqIntakeSpinDmd);
        //intakeSeqCarPitchDmdNTEntry.setDouble(intakeSeqCarouselPitchDmd);
        //intakeSeqCarTurnDmdNTEntry.setDouble(intakeSeqCarouselSpinDmd);
        //intakeSeqStateNTEntry.setDouble(intakeSequenceState);
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

        internalIntakeSeqInProg = false;
        
        // spin if solenoid is out

        if (Controls.Intake.SPIN_FORWARD() && solenoid.isExtended()) {
            //if (pitchEncoder.getDistance() > 0) {
                //pitchMotor.setPercentOutput(.75);
            //} else {
            spinner.setPercentOutput(1);
            //}
        } else if(Controls.Intake.SPIN_BACKWARD() && solenoid.isExtended()) {
            spinner.setPercentOutput(-1);
        } else {
            //JAS add the intake sequence here because it wasn't running in hailfire periodic ??
            if ( Controls.Intake.INTAKE_SEQ() && Hailfire.objHailfire.allowIntakeSeqShooter ) {
                if ( !Hailfire.objHailfire.intakeSeqInProg ) {
                    initIntakeSequence = true;
                }
                Hailfire.objHailfire.procIntakeSequence( initIntakeSequence );
                initIntakeSequence = false;
                spinner.setPercentOutput(Hailfire.objHailfire.intakeSeqIntakeSpinDmd);
            }
            else {
                spinner.setPercentOutput(0.0);
                // set output demands to zero.
                Hailfire.objHailfire.intakeSeqCarouselPitchDmd = 0.0;
                Hailfire.objHailfire.intakeSeqCarouselSpinDmd = 0.0;
                Hailfire.objHailfire.intakeSeqIntakeSpinDmd = 0.0;
                // set next time the sequence runs it will init.
                initIntakeSequence = true;
            }    
        }

        Hailfire.objHailfire.intakeSeqInProg = internalIntakeSeqInProg;

        // debug
        //System.out.println("Intake Output: " + intakeSeqIntakeSpinDmd);
        //System.out.println("Carousel Height Output: " + intakeSeqCarouselPitchDmd);
        //System.out.println("Carousel Turn Output: " + intakeSeqCarouselSpinDmd);
        intakeSeqIntakeSpinDmdNTEntry.setDouble(Hailfire.objHailfire.intakeSeqIntakeSpinDmd);
        intakeSeqCarPitchDmdNTEntry.setDouble(Hailfire.objHailfire.intakeSeqCarouselPitchDmd);
        intakeSeqCarTurnDmdNTEntry.setDouble(Hailfire.objHailfire.intakeSeqCarouselSpinDmd);
        intakeSeqStateNTEntry.setDouble(Hailfire.objHailfire.getIntakeSeqState());

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