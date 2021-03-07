package frc.robot.base.util;

import frc.robot.base.input.Axis;
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.StandardDriveTrain;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.Controls;

public class DriveUtil {
    
    // this stuff shouldn't be static but I don't think it'll ever matter and it's easiest for now

    private static double controllerDeadBand = 0.2;
    private static int controllerPower = 2;

    public static void standardDrive(StandardDriveTrain driveTrain, Controller controller, boolean reverse) {
        int r = reverse ? -1 : 1;
        double fb = - r * Util.adjustInput(controller.getAxis(Controls.DriveTrain.DRIVE_FORWARD_BACKWARD), controllerDeadBand, controllerPower);
        double lr = r * Util.adjustInput(controller.getAxis(Controls.DriveTrain.TURN_LEFT_RIGHT), controllerDeadBand, controllerPower);

        double left = fb - lr;
        double right = fb + lr;
        
        if (driveTrain.isClosedLoop()){
            driveTrain.setLeftVelocity(left * driveTrain.getAbsoluteMaxSpeed());
            driveTrain.setRightVelocity(right * driveTrain.getAbsoluteMaxSpeed());
        } else {
            driveTrain.setLeftPercentOutput(left);
            driveTrain.setRightPercentOutput(right);
        }

        if (controller.buttonPressed(Controls.DriveTrain.USE_CLOSED_LOOP)) {
            driveTrain.setClosedLoop(true);
        }

        if (controller.buttonPressed(Controls.DriveTrain.DONT_USE_CLOSED_LOOP)) {
            driveTrain.setClosedLoop(false);
        }

        if(controller.getAxis(Axis.LEFT_TRIGGER) > 0.5) {
            driveTrain.resetDistance();
        }
    }

    private static Trajectory trajectory = new Trajectory();
    private static long pathStartTime = 0;

    public static final double ftPerMeter = 3.28084;
    
    public static void followPath(StandardDriveTrain driveTrain) {
        var state = trajectory.sample(System.currentTimeMillis() - pathStartTime);
        // this math is taken from looking at how they do it in the RamseteCommand class
        // around line 140 they do stuff with toWheelSpeeds which is where the math is
        var accel = state.accelerationMetersPerSecondSq;
        var curve = state.curvatureRadPerMeter * accel;
        /* "The track width of the drivetrain. Theoretically, this is the distance
   *     between the left wheels and right wheels. However, the empirical value may be larger than
   *     the physical measured value due to scrubbing effects." taken from DifferentialDriveKinematics */
        // TODO: set this value correctly (keeping units in mind)
        var trackWidthMeters = 1;
        var leftSpeed = accel - trackWidthMeters / 2 * curve;
        var rightSpeed = accel + trackWidthMeters / 2 * curve;
        
        driveTrain.setLeftVelocity(leftSpeed * ftPerMeter);
        driveTrain.setRightVelocity(rightSpeed * ftPerMeter);
    }
    
    public static boolean finishedPath() {
        return System.currentTimeMillis() - pathStartTime >= trajectory.getTotalTimeSeconds() * 1000;
    }
    
    public static void startTrajectory(Trajectory t) {
        trajectory = t;
        pathStartTime = System.currentTimeMillis();
    }
}
