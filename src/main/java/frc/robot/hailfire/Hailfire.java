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
    
    public Hailfire() {
        this.setAutoActions(auto1);
        SmartDashboard.getEntry("Auto List").setStringArray(autoList);
        registerController(Controls.drive);
    //    registerController(Controls.shooter); // JPS, we got rid of the controller
        registerController(Controls.aux);
    }

    //JAS added intake sequence
    private boolean initIntakeSequence = true;
    private int intakeSequenceState = 0;
    private Timer intakeSeqTimer = new Timer();

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();

        if ( Controls.DriveTrain.SWITCH_CAM() ) {
        // if (Controls.aux.getPov(Pov.D_PAD) >= 0) {   //JAS use generic definition instead
            var cameraNum = NTHandler.getVisionEntry("cameraNumber");
            cameraNum.setValue(cameraNum.getDouble(-1) + 1);
        }

        //JAS add the intake sequence here because it requires both Intake and Shooter methods..!!
        if ( Controls.Intake.INTAKE_SEQ() && intake.allowIntakeSequence && shooter.allowIntakeSequence ) {
            procIntakeSequence( initIntakeSequence );
            initIntakeSequence = false;
        }
        else {
            // if sequence allowed but not selected set spinner output to zero.
            if ( intake.allowIntakeSequence ) {
                intake.setSpinnerOutput(0.0);
            }
            // if sequence allowed but not selected set carousel outputs to zero.
            // TODO may not need this one... 
            if ( shooter.allowIntakeSequence ) {
                shooter.setCarouselTurnMotor(0.0);
                shooter.setCarouselHeightMotor(0.0);
            }
            initIntakeSequence = true;
        }    

        Vision.update();
    }

    // =========================================================================================
    //JAS added
    //  process the state machine for the intake sequence
    // TODO make sure states can't wait keep the same state forever.. (driver just lets go of button0)
    private boolean procIntakeSequence( boolean firstStep ) {

        double desiredIntakeSpinnerOutput = 0.0;
        double desiredCarouselHeightOutput = 0.0;
        double desiredCarouselTurnOutput = 0.0;

        // tuning constants
        final double carouselDownSpeed = 0.125;
        final double intakeSpinSpeed = 1.0;
        final double carouselDownTime = 2.0;
        final double intakeSpinOffDelay = 0.2;
        final double carouselTurnSpeed = 0.7;

        if ( firstStep ) {
            intakeSequenceState = 0;
        }

        switch ( intakeSequenceState ) {

            // 0 - start timer, set carousel motor down
            case 0:
                intakeSequenceState = 1;
                desiredCarouselHeightOutput = carouselDownSpeed;
                intakeSeqTimer.start();
                break;

            // 1 - carousel motor down.  Has timer expired
            case 1:
                desiredCarouselHeightOutput = carouselDownSpeed;
                if ( intakeSeqTimer.hasElapsed( carouselDownTime) ) {
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
                intakeSequenceState = 0;
                break;

            // should never get here...
            // set known state.
            default:
                intakeSequenceState = 0;
                break;

        }

        // set intake spinner output.
        intake.setSpinnerOutput(desiredIntakeSpinnerOutput);
        // set carousel height adjust output
        shooter.setCarouselHeightMotor(desiredCarouselHeightOutput);
        // set carousel spin output
        shooter.setCarouselTurnMotor(desiredCarouselTurnOutput);
        
        return ( intakeSequenceState == 0 );
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
