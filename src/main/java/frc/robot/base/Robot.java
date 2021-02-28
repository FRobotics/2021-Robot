package frc.robot.base;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.base.action.Action;
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * A generic robot class to extend each year that automatically handles subsystems and input
 */
public abstract class Robot extends TimedRobot {

    private ArrayList<Subsystem> subsystems = new ArrayList<>();
    private ArrayList<Controller> controllers = new ArrayList<>();
    private AutoActionHandler autoActionHandler = new AutoActionHandler();

    private List<? extends Action> autoActions = List.of();
    /**
     * Returns a list of actions you want to run during auto; this is only called once
     */
    public void setAutoActions(List<? extends Action> actions) {
        this.autoActions = actions;
    }

    @Override
    public void robotInit() {
        NTHandler.init(this.subsystems);
    }

    @Override
    public void robotPeriodic() {
        subsystems.forEach(Subsystem::periodic);
        controllers.forEach(Controller::postPeriodic);
        NTHandler.update();
    }

    @Override
    public void autonomousInit() {
        subsystems.forEach(subsystem -> subsystem.onInit(RobotMode.AUTONOMOUS));
        this.autoActionHandler.startActionQueue(autoActions); // NOTE: the actions need to reset!
        // I have no idea what I meant by that but it works and doesn't seem to be important lol (used to be todo)
        // I'll come back at some point and figure out what I meant and either delete the comment or fix it
    }

    @Override
    public void autonomousPeriodic() {
       this.autoActionHandler.periodic();
    }

    @Override
    public void teleopInit() {
        subsystems.forEach(subsystem -> subsystem.onInit(RobotMode.TELEOP));
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        subsystems.forEach(subsystem -> subsystem.onInit(RobotMode.TEST));
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledInit() {
        subsystems.forEach(subsystem -> subsystem.onInit(RobotMode.DISABLED));
    }

    public <S extends Subsystem> S register(S subsystem) {
        this.subsystems.add(subsystem);
        return subsystem;
    }

    public Controller registerController(int port) {
        return this.registerController(new Controller(new Joystick(port)));
    }

    public Controller registerController(Controller c) {
        this.controllers.add(c);
        return c;
    }

}
