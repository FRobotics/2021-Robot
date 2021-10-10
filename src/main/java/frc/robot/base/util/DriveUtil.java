package frc.robot.base.util;

// import frc.robot.base.input.Axis;  //JAS not used
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.StandardDriveTrain;
import frc.robot.hailfire.subsystem.DriveTrain;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.base.Controls;
import frc.robot.base.NTHandler;

public class DriveUtil {
    
    // this stuff shouldn't be static but I don't think it'll ever matter and it's easiest for now

    private static double controllerDeadBand = 0.2;
    private static int controllerPower = 2;

    public static void standardDrive(StandardDriveTrain driveTrain, Controller controller, boolean reverse) {
        double r = reverse ? -1 : 1;
        double fb = - r * Util.adjustInput(controller.getAxis(Controls.DriveTrain.DRIVE_FORWARD_BACKWARD), controllerDeadBand, controllerPower);
        double lr = -0.9d * Util.adjustInput(controller.getAxis(Controls.DriveTrain.TURN_LEFT_RIGHT), controllerDeadBand, controllerPower);

        double left = fb - lr;
        double right = fb + lr;
        
        if (driveTrain.isClosedLoop()){
            driveTrain.setLeftVelocity(left * driveTrain.getAbsoluteMaxSpeed());
            driveTrain.setRightVelocity(right * driveTrain.getAbsoluteMaxSpeed());
        } else {
            driveTrain.setLeftPercentOutput(left);
            driveTrain.setRightPercentOutput(right);
        }

        if (controller.buttonPressed( Controls.DriveTrain.USE_CLOSED_LOOP)) {
            driveTrain.setClosedLoop(true);
        }

        if (controller.buttonPressed(Controls.DriveTrain.DONT_USE_CLOSED_LOOP)) {
            driveTrain.setClosedLoop(false);
        }

        if(controller.getAxis(Controls.DriveTrain.RESET_DIST) > 0.5) {
            driveTrain.resetDistance();
        }
    }

    private static Trajectory trajectory = new Trajectory();

    private static DifferentialDriveOdometry trajOdom;
    private static DifferentialDriveKinematics trajKine;
    private static RamseteController trajRamsete;

    private static DriveTrain driveTrain;

    private static double trajInitialLeft = 0.d;
    private static double trajInitialRight = 0.d;

    private static long pathStartTime = 0;

    private static double trajXErrorFt, trajYErrorFt, trajGyroErrorDeg = 0.d;

    private static boolean trajOnTarget = false;
    
    
    public static void followPath() {

        double angle = driveTrain.gyro.getAngle();
        double max = driveTrain.getCurrentMaxSpeed();
        double left = driveTrain.getLeftDistance();
        double right = driveTrain.getRightDistance();
        Trajectory.State desiredState = trajectory.sample(((double)(System.currentTimeMillis() - pathStartTime))*0.001d);
        
        Pose2d trajCurrentPosition = trajOdom.update(
            Rotation2d.fromDegrees(-angle),
            Units.feetToMeters(left - trajInitialLeft),
            Units.feetToMeters(right - trajInitialRight)
        );
        
        ChassisSpeeds trajChassisDmd = trajRamsete.calculate(trajCurrentPosition, desiredState);
        DifferentialDriveWheelSpeeds trajWheelDmds = trajKine.toWheelSpeeds(trajChassisDmd);
        
        trajWheelDmds.normalize(Units.feetToMeters(max));

        driveTrain.setLeftVelocity(Units.metersToFeet(trajWheelDmds.leftMetersPerSecond));
        driveTrain.setRightVelocity(Units.metersToFeet(trajWheelDmds.rightMetersPerSecond));
        
        Transform2d trajErrorPose = trajCurrentPosition.minus(desiredState.poseMeters);
        trajXErrorFt = Units.metersToFeet(trajErrorPose.getX());
        trajYErrorFt = Units.metersToFeet(trajErrorPose.getY());
        trajGyroErrorDeg = trajErrorPose.getRotation().getDegrees();

        trajOnTarget = trajRamsete.atReference();

        NTHandler.getRobotEntry("trajXErrorFt").setDouble(trajXErrorFt);
        NTHandler.getRobotEntry("trajYErrorFt").setDouble(trajYErrorFt);
        NTHandler.getRobotEntry("trajGyroErrorDeg").setDouble(trajGyroErrorDeg);
        NTHandler.getRobotEntry("pathtime").setDouble(System.currentTimeMillis() - pathStartTime);
        NTHandler.getRobotEntry("onTarget").setBoolean(trajOnTarget);
    }
    
    public static boolean finishedPath() {
        boolean trajOnTime = System.currentTimeMillis() - pathStartTime >= trajectory.getTotalTimeSeconds() * 1000;
        boolean trajOutTime = System.currentTimeMillis() - pathStartTime >= (trajectory.getTotalTimeSeconds() + 0.1d) * 1000;

        return (trajOnTime && trajOnTarget) || trajOutTime;
    }
    
    private static final double trackWidth = 24.d;
    public static void startTrajectory(Trajectory t, Gyro gyro, double left, double right, DriveTrain dt) {
        System.out.println("THIS IS RUNNING A TRAJECTORY");
        driveTrain = dt;
        gyro.reset();
        trajOdom = new DifferentialDriveOdometry(new Rotation2d(gyro.getAngle()));
        trajKine = new DifferentialDriveKinematics(Units.feetToMeters(trackWidth/12.d));
        trajRamsete = new RamseteController();

        //trajRamsete.setEnabled(false);

    
        trajInitialLeft = left;
        trajInitialRight = right;

        trajectory = t;
        pathStartTime = System.currentTimeMillis();
    }
}
