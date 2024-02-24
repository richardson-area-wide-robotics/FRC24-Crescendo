package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LockMode;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.Pivot;

public class Lock extends Command {

    DriveSubsystem m_drive;
    DoubleSupplier sideways;
    DoubleSupplier forward;
    Pivot m_pivot;
    Camera m_camera;

    // PID controller for yawRate
    final PIDController yawRateController = new PIDController(
            Constants.ModuleConstants.kVisionTurningPIDGains.P,
            Constants.ModuleConstants.kVisionTurningPIDGains.I,
            Constants.ModuleConstants.kVisionTurningPIDGains.D);
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private LockMode mode = LockMode.SPEAKER_LOCK_MODE;
    private boolean isCanceled;

    /**
     *
     * @param drive    passes controller
     * @param camera   passes camera
     * @param forward  passes y translation
     * @param sideways passes x translation
     */
    public Lock(DriveSubsystem drive, Pivot pivot, Camera camera, DoubleSupplier forward, DoubleSupplier sideways) {
        this.m_drive = drive;
        this.forward = forward;
        this.sideways = sideways;
        this.m_pivot = pivot;
        this.m_camera = camera;
        this.addRequirements(m_drive, m_pivot);
    }

    @Override
    public void initialize() {

        yawRateController.reset();
    }

    @Override
    public boolean isFinished() {
        return this.isCanceled;
    }

    public void setMode(LockMode mode) {
        this.mode = mode;
    }

    public void endCommand() {
        this.isCanceled = false;
    }

    // Returns -1 when trying to intake
    private int getAprilTagForModeAndAlliance() {

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                switch (this.mode) {
                    case SPEAKER_LOCK_MODE: {
                        return 4;
                    }
                    case AMP_LOCK_MODE: {
                        return 5;
                    }
                    case GAMEPIECE_LOCK_MODE:
                    default: {
                        return -1;
                    }
                }
            } else {
                switch (this.mode) {
                    case SPEAKER_LOCK_MODE: {
                        return 7;
                    }
                    case AMP_LOCK_MODE: {
                        return 6;
                    }
                    case GAMEPIECE_LOCK_MODE:
                    default: {
                        return -1;
                    }
                }
            }
        }

        return -1;
    }

    // This sets the yawRate to circle the desired object while maintaning driver
    // controll of motion
    @Override
    public void execute() {

        final int aprilTagId = getAprilTagForModeAndAlliance();
        Pose3d robotPose = new Pose3d(m_drive.getPose());
        
        SmartDashboard.putString("pose", robotPose.toString());

        Measure<Angle> angularOffset = Degrees.of(0.0);
        Measure<Angle> pivotAngle =  Degrees.of(0.0);

        // Optional<EstimatedRobotPose> aprilTagBasedRobotPose = m_camera.getEstimatedGlobalPose();
        // if (aprilTagBasedRobotPose.isPresent()) {
            // Pose3d robotPose = aprilTagBasedRobotPose.get().estimatedPose;
            // SmartDashboard.putBoolean("photonPose", true);
            
             pivotAngle = getPitchAngleToAprilTag(robotPose, aprilTagId);
            angularOffset = getYawAngleToAprilTag(robotPose, aprilTagId);
                    double yawRate = yawRateController.calculate(angularOffset.in(Units.Radians), 0.0) * 0.3;

        SmartDashboard.putNumber("yawRate", yawRate);
        SmartDashboard.putNumber("angularOffset", angularOffset.in(Degrees));

        SmartDashboard.putNumber("aprilTag", aprilTagId);

        // limits robot max speed while in locked-on mode
        double limitedForward = Math.max(forward.getAsDouble(), -1.0 * Constants.ModuleConstants.MAX_LOCKED_ON_SPEED);
        limitedForward = Math.min(limitedForward, Constants.ModuleConstants.MAX_LOCKED_ON_SPEED);
        double limitedSideways = Math.max(sideways.getAsDouble(), -1.0 * Constants.ModuleConstants.MAX_LOCKED_ON_SPEED);
        limitedSideways = Math.min(limitedSideways, Constants.ModuleConstants.MAX_LOCKED_ON_SPEED);

        m_drive.drive(limitedForward, limitedSideways, yawRate, true);
        SmartDashboard.putNumber("pitch angle", pivotAngle.in(Degrees));
        // }


        // m_pivot.pivotTo(pivotAngle);
    }

    public Measure<Angle> getYawAngleToAprilTag(Pose3d currentRobotPoseField, int tagId) {

        Pose3d speakerPose3d = aprilTagFieldLayout.getTagPose(tagId).get();
        Translation3d robotToSpeaker = speakerPose3d.getTranslation().minus(currentRobotPoseField.getTranslation());

        double angle_rad = Math.atan2(robotToSpeaker.getY(), robotToSpeaker.getX());
        return Units.Radians.of(angle_rad);
    }

    public Measure<Angle> getPitchAngleToAprilTag(Pose3d currentRobotPoseField, int tagId) {

        Pose3d speakerPose3d = aprilTagFieldLayout.getTagPose(tagId).get();
        Translation3d robotToSpeaker = speakerPose3d.getTranslation().minus(currentRobotPoseField.getTranslation());

        double distanceMeters = Math.sqrt(
                (robotToSpeaker.getX() * robotToSpeaker.getX()) + (robotToSpeaker.getY() * robotToSpeaker.getY()));
        double angle_rad = Math.atan2(Constants.ShooterConstants.speakerHeight.in(Meters), distanceMeters);
        return Units.Radians.of(angle_rad);
    }

}