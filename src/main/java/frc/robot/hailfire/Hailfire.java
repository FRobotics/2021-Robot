package frc.robot.hailfire;

import frc.robot.base.NTHandler;
import frc.robot.base.Robot;
import frc.robot.base.action.Action;
import frc.robot.base.action.SetupAction;
import frc.robot.base.action.TimedAction;
import frc.robot.base.input.Pov;
import frc.robot.base.util.PosControl;
import frc.robot.base.util.DriveUtil;
import frc.robot.base.util.CommandList;
import frc.robot.base.util.Command;
import frc.robot.hailfire.subsystem.Climber;
import frc.robot.hailfire.subsystem.DriveTrain;
import frc.robot.hailfire.subsystem.Intake;
import frc.robot.hailfire.subsystem.Shooter;
import frc.robot.base.input.Controller;
import frc.robot.base.device.Pixy;
import edu.wpi.first.wpilibj.Timer;  //jas added
import edu.wpi.first.networktables.NetworkTableEntry; //jas added


import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class Hailfire extends Robot {
    
    private final CommandList commandList = new CommandList(
        List.<Command>of(
            new Command(Command.CommandType.BALL_PICKUP, 0.0, 0.0, 0.0, 0.0),
            new Command(Command.CommandType.DRIVE, 5, 0.1, 0.0, 0.0)
        )
    );

//     from repos directoy repository -- never updated to github
//      
//    private final CommandList commandList = new CommandList(
//        List.<Command>of(
//            new Command(Command.CommandType.SLEEP, 200, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, 20.0, 0.5, 0.0, 0.0),
//            new Command(Command.CommandType.TURN, 90.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, 3.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.TURN, -90.0, 0.0, 0.0, 0.0),
//            /*new Command(Command.CommandType.SHOOT, 3.0, 5.0, 0.0, 0.0), */
//            new Command(Command.CommandType.TURN, -90.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, 3.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.TURN, -90.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, 50.0, 0.5, 0.0, 0.0)
//        )
//    );
//    private final CommandList leftCommands = new CommandList(
//        List.<Command>of(
//            new Command(Command.CommandType.SLEEP, 200, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, 20.0, 0.5, 0.0, 0.0),
//            new Command(Command.CommandType.TURN, 90.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, 3.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.TURN, -90.0, 0.0, 0.0, 0.0),
//            /*new Command(Command.CommandType.SHOOT, 3.0, 5.0, 0.0, 0.0), */
//            new Command(Command.CommandType.TURN, -90.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, 3.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.TURN, -90.0, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, 50.0, 0.5, 0.0, 0.0)
//        )
//    );
//    private final CommandList middleCommands = new CommandList(
//        List.<Command>of(
//            new Command(Command.CommandType.SLEEP, 200, 0.0, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, -7.0, 0.5, 0.0, 0.0),
//            new Command(Command.CommandType.TURN, 17.0, 0.5, 0.0, 0.0),
//            /*new Command(Command.CommandType.SHOOT, 3.0, 0.5, 0.0, 0.0),*/
//            new Command(Command.CommandType.TURN, -17.0, 0.5, 0.0, 0.0),
//            new Command(Command.CommandType.DRIVE, -17.0, 0.5, 0.0, 0.0),
//            new Command(Command.CommandType.BALL_PICKUP, 0.0, 0.0, 0.0, 0.0)
//        )
//     );

    public final DriveTrain driveTrain = register(new DriveTrain());
    public final Shooter shooter = register(new Shooter());
    public final Intake intake = register(new Intake());
    public final Climber climber = register(new Climber());

    private PosControl drivePosControl = new PosControl(10, 1, 0.1, 0.5, 5);


    //JAS added intake sequence
    private boolean initIntakeSequence = true;
    private int intakeSequenceState = 0;
    private Timer intakeSeqTimer = new Timer();
    public static double intakeSeqCarouselPitchDmd = 0.0;
    public static double intakeSeqCarouselSpinDmd = 0.0;
    public static double intakeSeqIntakeSpinDmd = 0.0;
    public static boolean intakeSeqInProg = false;
    private NetworkTableEntry intakeSeqCarPitchDmdNTEntry;
    private NetworkTableEntry intakeSeqCarTurnDmdNTEntry;
    private NetworkTableEntry intakeSeqIntakeSpinDmdNTEntry;
    private NetworkTableEntry intakeSeqStateNTEntry;
    
    //jas add global variable for camera number
    private int switchedCamNumber = 0;   // toggles between 0 and 1.
    private NetworkTableEntry cameraNumNTEntry;

    
    public Hailfire() {

        this.setAutoActions(auto1);
        SmartDashboard.getEntry("Auto List").setStringArray(autoList);
        registerController(Controls.drive);
    //    registerController(Controls.shooter); // JPS, we got rid of the controller
        registerController(Controls.aux);

        cameraNumNTEntry = NTHandler.getVisionEntry("cameraNumber");    // switched cam number
        double dblValue = switchedCamNumber;    
        cameraNumNTEntry.setDouble( dblValue ); //set initial value

        //jas added intake seq
        intakeSeqCarPitchDmdNTEntry = NTHandler.getRobotEntry("IntakeSeq/CarPitchDmd");
        intakeSeqCarTurnDmdNTEntry = NTHandler.getRobotEntry("IntakeSeq/CarTurnDmd");
        intakeSeqIntakeSpinDmdNTEntry = NTHandler.getRobotEntry("IntakeSeq/IntakeSpinDmd");
        intakeSeqStateNTEntry = NTHandler.getRobotEntry("IntakeSeq/State");
        intakeSeqIntakeSpinDmdNTEntry.setDouble(intakeSeqIntakeSpinDmd);
        intakeSeqCarPitchDmdNTEntry.setDouble(intakeSeqCarouselPitchDmd);
        intakeSeqCarTurnDmdNTEntry.setDouble(intakeSeqCarouselSpinDmd);
        intakeSeqStateNTEntry.setDouble(intakeSequenceState);
    
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();

        if ( Controls.DriveTrain.SWITCH_CAM() ) {
        // if (Controls.aux.getPov(Pov.D_PAD) >= 0) {   //JAS use generic definition instead
            switchedCamNumber = 1 - switchedCamNumber;  //jas toggle between 0 and 1.
            double dblValue = switchedCamNumber;   
            cameraNumNTEntry.setValue( dblValue );
        }

        //JAS add the intake sequence here because it requires both Intake and Shooter methods..!!
        if ( Controls.Intake.INTAKE_SEQ() && intake.allowIntakeSequence && shooter.allowIntakeSequence ) {
            procIntakeSequence( initIntakeSequence );
            initIntakeSequence = false;
            intakeSeqInProg = true;
        }
        else {
            intakeSeqInProg = false;
            // set output demands to zero.
            intakeSeqCarouselPitchDmd = 0.0;
            intakeSeqCarouselSpinDmd = 0.0;
            intakeSeqIntakeSpinDmd = 0.0;
            // set next time the sequence runs it will init.
            initIntakeSequence = true;
        }    
        // debug
        //System.out.println("Intake Output: " + intakeSeqIntakeSpinDmd);
        //System.out.println("Carousel Height Output: " + intakeSeqCarouselPitchDmd);
        //System.out.println("Carousel Turn Output: " + intakeSeqCarouselSpinDmd);
        intakeSeqIntakeSpinDmdNTEntry.setDouble(intakeSeqIntakeSpinDmd);
        intakeSeqCarPitchDmdNTEntry.setDouble(intakeSeqCarouselPitchDmd);
        intakeSeqCarTurnDmdNTEntry.setDouble(intakeSeqCarouselSpinDmd);
        intakeSeqStateNTEntry.setDouble(intakeSequenceState);


        Vision.update();
    }

    // =========================================================================================
    //JAS added
    //  process the state machine for the intake sequence
    // TODO could add timeout to conditions that could wait forever.. however driver just releases button.
    private boolean procIntakeSequence( boolean firstStep ) {

        double desiredIntakeSpinnerOutput = 0.0;
        double desiredCarouselHeightOutput = 0.0;
        double desiredCarouselTurnOutput = 0.0;

        // tuning constants
        final double carouselUpSpeed = -1.0;
        final double intakeSpinSpeed = 1.0;
        final double carouselUpTime = 2.5;
        final double intakeSpinOffDelay = 0.5;
        final double carouselTurnSpeed = 0.7;

        if ( firstStep ) {
            intakeSequenceState = 0;
        }

        switch ( intakeSequenceState ) {

            // 0 - start timer, raise the carousel
            case 0:
                intakeSequenceState = 1;
                desiredCarouselHeightOutput = carouselUpSpeed;
                intakeSeqTimer.reset();
                intakeSeqTimer.start();
                break;

            // 1 - continue raising the carousel.  Has timer expired?
            case 1:
                desiredCarouselHeightOutput = carouselUpSpeed;
                if ( intakeSeqTimer.hasElapsed( carouselUpTime) ) {
                    intakeSequenceState = 2;
                }
                break;

            // 2 - spin intake, look for ball detect.
            case 2:
                intakeSeqTimer.stop();
                desiredIntakeSpinnerOutput = intakeSpinSpeed;
                if ( intake.ballDetectSensor ) {
                    intakeSequenceState = 3;
                }
                break;

            // 3 - spin intake - start timer
            case 3:
                desiredIntakeSpinnerOutput = intakeSpinSpeed;
                intakeSeqTimer.reset();
                intakeSeqTimer.start();
                intakeSequenceState = 4;
                break;

            // 4 - spin intake - check timer
            case 4:
                desiredIntakeSpinnerOutput = intakeSpinSpeed;
                if ( intakeSeqTimer.hasElapsed( intakeSpinOffDelay ) ) {
                    intakeSequenceState = 5;
                }
                break;

            // 5 - stop spin, start rotate
            case 5:
                desiredCarouselTurnOutput = carouselTurnSpeed;
                intakeSeqTimer.stop();
                intakeSequenceState = 6;
                break;

            // 6 - rotate, look for limit switch off..
            case 6:
                desiredCarouselTurnOutput = carouselTurnSpeed;
                if ( shooter.getCarouselTurnLimitSwitch() ) {
                    intakeSequenceState = 7;
                }
                break;

            // 7 - rotate, look for limit switch off..
            case 7:
                desiredCarouselTurnOutput = carouselTurnSpeed;
                if ( shooter.getCarouselTurnLimitSwitch() ) {
                    intakeSequenceState = 8;
                }
                break;

            // 8 - stop rotate.
            case 8:
                intakeSequenceState = 9;
                break;
            
            // 9 - DONE
            case 9:
                break;

            // should never get here...
            // set known state.
            default:
                intakeSequenceState = 9;
                break;

        }

        // set intake spinner output.
        intakeSeqIntakeSpinDmd = desiredIntakeSpinnerOutput;
        // set carousel height adjust output
        intakeSeqCarouselPitchDmd = desiredCarouselHeightOutput;
        // set carousel spin output
        intakeSeqCarouselSpinDmd = desiredCarouselTurnOutput;

        //System.out.println("Sequence state: " + intakeSequenceState);
    
        return ( intakeSequenceState == 9 );
    }

    // =========================================================================================

    private final String[] autoList = new String[]{
        "None", "Auto 1", "Trajectory Test", "Right Motor Test", "Command List", "Slalom", "BounceTogether", "Bounce1", "Bounce2","Bounce3","Bounce4", "Barrel Race"
    };

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();

        switch(SmartDashboard.getEntry("Auto Selector").getString("")) {
            default:
                setAutoActions(List.of());
                break;
            case "Auto 1":
                setAutoActions(auto1);
                break;
            case "Trajectory Test":
                setAutoActions(auto2);
                break;
            case "Right Motor Test":
                setAutoActions(auto3);
                break;
            case "Command List":
                setAutoActions(autoCommands);
                break;
            case "Slalom":
                setAutoActions(autoSlalom);
                break;
            case "Bounce1":
                setAutoActions(autoBounce1);
                break;
            case "Bounce2":
                setAutoActions(autoBounce2);
                break;
            case "Bounce3":
                setAutoActions(autoBounce3);
                break;
            case "Bounce4":
                setAutoActions(autoBounce4);
                break;
            case "BounceTogether":
                setAutoActions(autoBounceTogether);
                break;
            case "Barrel Race":
                setAutoActions(autoBarrel);
                break;
        }
    }

    // Note: I mention this elsewhere but the way I set this up is kinda awkward / repetitive
    // it works well though and isn't too repetitive but I might wanna change it in the future

    // old auto
    private final List<? extends Action> auto1 = List.of(
            new SetupAction(() -> driveTrain.startAction(
                    new Action(
                            () -> {
                                driveTrain.setPercentOutput(drivePosControl.getSpeed(driveTrain.getAverageDistance())/5.5);
                            },
                            drivePosControl::isFinished
                    )
            ), driveTrain::isFinished),
            new TimedAction(2000),
            new SetupAction(() -> shooter.startAction(
                    new TimedAction(
                            () -> shooter.shoot(false),
                            7000
                    )
            ), shooter::isFinished)
    );

    // follow path
    // note: if the trajectory isn't set / couldn't load it should theoretically just do nothing
    private final List<? extends Action> auto2 = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.SLALOM,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
    private final List<? extends Action> autoSlalom = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.SLALOM,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
    private final List<? extends Action> autoBounceTogether = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BOUNCE1,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BOUNCE2,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BOUNCE3,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BOUNCE4,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
    //  AUTONOMOUS LIST FOR BOUNCE CHALLENGE
    private final List<? extends Action> autoBounce1 = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BOUNCE1,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
    private final List<? extends Action> autoBounce2 = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BOUNCE2,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
    private final List<? extends Action> autoBounce3 = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BOUNCE3,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
    private final List<? extends Action> autoBounce4 = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BOUNCE4,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
    private final List<? extends Action> autoBarrel = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> DriveUtil.startTrajectory(
                    driveTrain.BARREL,
                    driveTrain.gyro,
                    driveTrain.getLeftDistance(),
                    driveTrain.getRightDistance(),
                    driveTrain
                )
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath();},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
    private final List<? extends Action> autoCommands = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> commandList.Init()
            )
        )),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> commandList.Execute(this),
                commandList::Finished
            )
        ), commandList::Finished)
    );

    private final List<? extends Action> auto3 = List.of(
        new TimedAction(() -> driveTrain.startAction(
            new Action(() -> {
                driveTrain.setLeftVelocity(0);
                driveTrain.setRightVelocity(1);
            })
        ), 3000)
    );

// --- from repos directory - never updated to github...  
//
    /*private final List<? extends Action> autoCommands = List.of(
        new SetupAction(() -> driveTrain.startAction(
            new SetupAction(
                () -> commandList.Init()
            )
        )),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> commandList.Execute(this),
                commandList::Finished
            )
        ), commandList::Finished)
    );*/    
}
