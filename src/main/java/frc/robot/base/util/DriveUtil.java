package frc.robot.base.util;

import frc.robot.base.input.Axis;
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.StandardDriveTrain;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.base.Controls;
import frc.robot.base.NTHandler;

public class DriveUtil {

    // this stuff shouldn't be static but I don't think it'll ever matter and it's
    // easiest for now

    private static double controllerDeadBand = 0.2;
    private static int controllerPower = 2;

    public static void standardDrive(StandardDriveTrain driveTrain, Controller controller, boolean reverse) {
        double r = reverse ? -1 : 1;
        double fb = -r * Util.adjustInput(controller.getAxis(Controls.DriveTrain.DRIVE_FORWARD_BACKWARD),
                controllerDeadBand, controllerPower);
        double lr = -Util.adjustInput(controller.getAxis(Controls.DriveTrain.TURN_LEFT_RIGHT), controllerDeadBand,
                controllerPower);

        double left = fb - lr;
        double right = fb + lr;

        if (driveTrain.isClosedLoop()) {
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

        if (controller.getAxis(Axis.LEFT_TRIGGER) > 0.5) {
            driveTrain.resetDistance();
        }
    }

    private static Trajectory trajectory = new Trajectory();

    private static DifferentialDriveOdometry trajOdom;
    private static DifferentialDriveKinematics trajKine;
    private static RamseteController trajRamsete;

    private static double trajInitialGyro;
    private static double trajInitialLeft = 0.d;
    private static double trajInitialRight = 0.d;

    private static long pathStartTime = 0;

    private static double trajXErrorFt, trajYErrorFt, trajGyroErrorDeg = 0.d;

    private static boolean trajOnTarget = false;

    public static void followPath(StandardDriveTrain driveTrain, double angle, double max) {
        double left = driveTrain.getLeftDistance();
        double right = driveTrain.getRightDistance();
        Trajectory.State currentState = trajectory.sample((System.currentTimeMillis() - pathStartTime) / 1000.d);

        Pose2d trajCurrentPosition = trajOdom.update(Rotation2d.fromDegrees(angle - trajInitialGyro),
                Units.feetToMeters(left - trajInitialLeft), Units.feetToMeters(right - trajInitialRight));

        //trajRamsete.setEnabled(false);
        ChassisSpeeds trajChassisDmd = trajRamsete.calculate(trajCurrentPosition, currentState);
        DifferentialDriveWheelSpeeds trajWheelDmds = trajKine.toWheelSpeeds(trajChassisDmd);

        trajWheelDmds.normalize(Units.feetToMeters(max));

        driveTrain.setLeftVelocity(Units.metersToFeet(trajWheelDmds.leftMetersPerSecond));
        driveTrain.setRightVelocity(Units.metersToFeet(trajWheelDmds.rightMetersPerSecond));

        Transform2d trajErrorPose = trajCurrentPosition.minus(currentState.poseMeters);
        trajXErrorFt = Units.metersToFeet(trajErrorPose.getX());
        trajYErrorFt = Units.metersToFeet(trajErrorPose.getY());
        trajGyroErrorDeg = trajErrorPose.getRotation().getDegrees();

        trajOnTarget = trajRamsete.atReference();

        NTHandler.getRobotEntry("trajXErrorFt").setDouble(trajXErrorFt);
        NTHandler.getRobotEntry("trajYErrorFt").setDouble(trajYErrorFt);
        NTHandler.getRobotEntry("trajGyroErrorDeg").setDouble(trajGyroErrorDeg);

        NTHandler.getRobotEntry("posX").setDouble(trajCurrentPosition.getX());
        NTHandler.getRobotEntry("posY").setDouble(trajCurrentPosition.getY());
        NTHandler.getRobotEntry("currentTrajX").setDouble(currentState.poseMeters.getX());
        NTHandler.getRobotEntry("currentTrajY").setDouble(currentState.poseMeters.getY());
        //NTHandler.getRobotEntry("header").setDouble(currentState.poseMeters.getRotation().getDegrees());
    }

    public static boolean finishedPath() {

        double currentTime = System.currentTimeMillis() - pathStartTime;
        boolean trajOnTime = currentTime >= trajectory.getTotalTimeSeconds() * 1000;
        boolean trajOutTime = currentTime >= (trajectory.getTotalTimeSeconds() + 10.d) * 1000;
        NTHandler.getRobotEntry("trajTotalTime").setDouble(trajectory.getTotalTimeSeconds());
        NTHandler.getRobotEntry("trajCurrentTime").setDouble(currentTime / 1000);

        return (trajOnTime && trajOnTarget) || trajOutTime;
    }

    private static final double trackWidth = 24.d;

    public static void startTrajectory(Trajectory t, double gyroAngle, double left, double right) {
        trajOdom = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
        trajKine = new DifferentialDriveKinematics(Units.feetToMeters(trackWidth));
        trajRamsete = new RamseteController();

        trajInitialGyro = gyroAngle;
        trajInitialLeft = left;
        trajInitialRight = right;

        trajectory = t;
        pathStartTime = System.currentTimeMillis();
    }
}
