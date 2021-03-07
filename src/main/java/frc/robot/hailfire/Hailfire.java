package frc.robot.hailfire;

import frc.robot.base.NTHandler;
import frc.robot.base.Robot;
import frc.robot.base.action.Action;
import frc.robot.base.action.SetupAction;
import frc.robot.base.action.TimedAction;
import frc.robot.base.input.Pov;
import frc.robot.base.util.PosControl;
import frc.robot.base.util.DriveUtil;
import frc.robot.hailfire.subsystem.Climber;
import frc.robot.hailfire.subsystem.DriveTrain;
import frc.robot.hailfire.subsystem.Intake;
import frc.robot.hailfire.subsystem.Shooter;
import frc.robot.base.input.Controller;
import frc.robot.base.device.Pixy;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class Hailfire extends Robot {
    
    private final DriveTrain driveTrain = register(new DriveTrain());
    private final Shooter shooter = register(new Shooter());
    private final Intake intake = register(new Intake());
    private final Climber climber = register(new Climber());
    
    private PosControl drivePosControl = new PosControl(10, 1, 0.1, 0.5, 5);
    
    public Hailfire() {
        this.setAutoActions(auto1);
        SmartDashboard.getEntry("Auto List").setStringArray(autoList);
        registerController(Controls.drive);
        registerController(Controls.shooter);
        registerController(Controls.aux);
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();

        if (Controls.aux.getPov(Pov.D_PAD) >= 0) {
            var cameraNum = NTHandler.getVisionEntry("cameraNumber");
            cameraNum.setValue(cameraNum.getDouble(-1) + 1);
        }

        Vision.update();
    }

    private final String[] autoList = new String[]{
        "None", "Auto 1", "Trajectory Test"
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
                () -> DriveUtil.startTrajectory(driveTrain.TURN_RIGHT)
            )
        ), driveTrain::isFinished),
        new SetupAction(() -> driveTrain.startAction(
            new Action(
                () -> {DriveUtil.followPath(driveTrain);},
                DriveUtil::finishedPath
            )
        ), driveTrain::isFinished)
    );
}
