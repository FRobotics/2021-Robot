package frc.robot.hailfire;

import frc.robot.base.NTHandler;
import frc.robot.base.Robot;
import frc.robot.base.action.Action;
import frc.robot.base.action.SetupAction;
import frc.robot.base.action.TimedAction;
import frc.robot.base.input.Pov;
import frc.robot.base.util.PosControl;
import frc.robot.hailfire.subsystem.Climber;
import frc.robot.hailfire.subsystem.DriveTrain;
import frc.robot.hailfire.subsystem.Intake;
import frc.robot.hailfire.subsystem.Shooter;
import frc.robot.base.input.Controller;

import java.util.List;

@SuppressWarnings("unused")
public class Hailfire extends Robot {

    private final Controller driveController = registerController(0);
    private final Controller shooterController = registerController(1);
    private final Controller auxiliaryController = registerController(2);

    private final DriveTrain driveTrain = register(new DriveTrain(driveController));
    private final Shooter shooter = register(new Shooter(shooterController));
    private final Intake intake = register(new Intake(auxiliaryController));
    private final Climber climber = register(new Climber(auxiliaryController));

    private PosControl drivePosControl = new PosControl(10, 2, 0.1, 0.5, 5);

    public Hailfire() {
        this.setAutoActions(auto1);
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();

        if (auxiliaryController.getPov(Pov.D_PAD) >= 0) {
            var cameraNum = NTHandler.getVisionEntry("cameraNumber");
            cameraNum.setValue(cameraNum.getDouble(-1) + 1);
        }

        Vision.update();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();

        switch((int)NTHandler.getRobotEntry("autoProgram").getDouble(0)) {
            default:
                setAutoActions(List.of());
                break;
            case 1:
                setAutoActions(auto1);
                break;
            case 2:
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
                            () -> driveTrain.setVelocity(drivePosControl.getSpeed(-driveTrain.getAverageDistance())),
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
        // TODO: file path there
        new SetupAction(() -> driveTrain.initTrajectory("FILE PATH HERE") , () -> driveTrain.startAction(
            new Action(
                driveTrain::followPath,
                driveTrain::finishedPath
            )
        ), driveTrain::isFinished)
    );
}
