package frc.robot.base.util;

import java.util.List;
//import edu.wpi.first.wpilibj.DigitalInput; // JPS it was never used

import frc.robot.hailfire.Hailfire;

public class CommandList {
    private List<Command> commands;
    private Command currentCommand;

    private boolean finished = false;
    private int step = 0;
    private Boolean initialize = true;
    //private DigitalInput ballIntake;

    public boolean Finished() {
        return finished;
    }
    public void Init() { 
        finished = false;
        step = 0;
        initialize = true;
        currentCommand = commands.get(step);
    }
    public void NextStep() {
        step++;
        initialize = true;
        currentCommand = commands.get(step);   
    }
    public CommandList(List<Command> commands) {
        this.commands = commands;
        //ballIntake = new DigitalInput(2);
        step = 0;
    }
    public void Execute(Hailfire hailfire) {
        if (currentCommand == null) {
            currentCommand = commands.get(step);
        }
        if (step >= commands.size()) {
            finished = true;
            initialize = true;
            return;
        }
        
        switch(currentCommand.type) {
        case GOTO:
            step = (int)currentCommand.arguments[0];
            break;
        case DRIVE:
            double currentDist = hailfire.driveTrain.getLeftDistance();
            if (initialize) {
                currentCommand.arguments[3] = currentDist + currentCommand.arguments[0];
                initialize = false;
            }
                
            hailfire.driveTrain.autoDriveForward(currentCommand.arguments[3] - currentDist);

            if (currentCommand.arguments[3] - currentDist <= currentCommand.arguments[1]) {
                NextStep();
                hailfire.driveTrain.setLeftPercentOutput(0);
                hailfire.driveTrain.setRightPercentOutput(0);
            }
                
            break;
        case SLEEP:
            // USAGE: SLEEP ( milliseconds )
            if (initialize) {
                currentCommand.arguments[3] = System.currentTimeMillis();
                initialize = false;
            }
            if (System.currentTimeMillis() >= currentCommand.arguments[3] + currentCommand.arguments[0]) {
                initialize = true;
                step++;
            }
            break;
        case TURN:
            // USAGE: TURN ( amtInDegrees, threshold )
            double currentAngle = hailfire.driveTrain.gyro.getAngle();
            if (initialize) {
                currentCommand.arguments[3] = hailfire.driveTrain.gyro.getAngle();
                initialize = false;
            }
            double targetAngle = currentCommand.arguments[3] - currentCommand.arguments[0];
            
            hailfire.driveTrain.turnToAngle(targetAngle);
            if (Math.abs(currentAngle - targetAngle) <= currentCommand.arguments[1]) {   
                    hailfire.driveTrain.setLeftPercentOutput(0);
                    hailfire.driveTrain.setRightPercentOutput(0);
                    step++;
                    initialize = true;
            }
            break;
        case PIXYGOTO:
            initialize = false;
            double pixyOut = hailfire.driveTrain.ReadPixy();
            System.out.println(pixyOut);
            if (pixyOut == -1000.0d) {
                initialize = true;
                step = (int)currentCommand.arguments[1];
            } else if (pixyOut < currentCommand.arguments[0]) {
                initialize = true;
                step = (int)currentCommand.arguments[2];
            }else if (pixyOut >= currentCommand.arguments[0]) {
                initialize = true;
                step = (int)currentCommand.arguments[3];
            }
        case BALL_PICKUP:
            if (initialize) {
                currentCommand.arguments[3] = 0.0;
                currentCommand.arguments[2] = 0.0;
                currentCommand.arguments[1] = 0.0;
                currentCommand.arguments[0] = 0.0;
                hailfire.intake.solenoid.extend();

                initialize = false;
            }
            System.out.println((int)currentCommand.arguments[3]);
            switch ((int)currentCommand.arguments[3]) {
            case 0:
                hailfire.intake.spinner.setPercentOutput(1.0);
                if (hailfire.intake.sensor.get()) {
                    currentCommand.arguments[3]++;
                    currentCommand.arguments[2] = System.currentTimeMillis();
                }
                break;
            case 1:
                System.out.println(System.currentTimeMillis() - currentCommand.arguments[2]);
                if (System.currentTimeMillis() - currentCommand.arguments[2] > 1000) {
                    currentCommand.arguments[3]++; 
                    hailfire.intake.spinner.setPercentOutput(0);
                }
                break;
            case 2:
                hailfire.shooter.carousel.setPercentOutput(0.7);
                if (hailfire.shooter.carouselSwitch.get()) {
                    hailfire.shooter.carousel.setPercentOutput(0);
                    currentCommand.arguments[3]++;
                }
                break;
            case 3:
                hailfire.intake.solenoid.retract();
                NextStep();
                break;
            }
            break;
        default:
            break;
        }
    }
}