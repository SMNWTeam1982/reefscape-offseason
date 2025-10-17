package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Implements a Vision Subsystem for Photon Vision */
public class PhotonVisionSubsystem extends VisionSubsystem {

    public static class PhotonVisionConstants {
        public static final String PHOTON_CAMERA_NAME = "limelight-front";

        public static final Matrix<N3, N1> PHOTON_CAM_VISION_TRUST = VecBuilder.fill(0.5, 0.5, 1);

        public static final Transform3d PHOTON_CAM_RELATIVE_TO_ROBOT = new Transform3d(
                new Translation3d(Units.inchesToMeters(12.0), Units.inchesToMeters(0.0), Units.inchesToMeters(9.75)),
                new Rotation3d(0.0, 10.0, 0.0));
    }

    private final PhotonCamera instanceCamera;

    private final PhotonPoseEstimator photonPoseEstimator;

    private final String cameraName;

    public PhotonVisionSubsystem(Transform3d cameraRelativeToRobot, String cameraName) {

        photonPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                PoseStrategy.LOWEST_AMBIGUITY,
                cameraRelativeToRobot);
        instanceCamera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
    }

    @Override
    public String getName() {
        return cameraName;
    }

    @Override
    protected Optional<VisionData> getVisionResult() {
        Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

        for (var result : instanceCamera.getAllUnreadResults()) {
            lastEstimatedPose = photonPoseEstimator.update(result);
        } // if getAllUnreadResults() is empty then lastEstimatedPose will be Optional.empty()
        // this also accounts for results that have data but are surrounded by results without data

        if (lastEstimatedPose.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose estimatedPose = lastEstimatedPose.get();

        return Optional.of(new VisionData(
                estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds,
                PhotonVisionConstants.PHOTON_CAM_VISION_TRUST // we should calculate this the same way photonVision does
                // in their example code
                ));
    }

    @Override
    public Command resetPose(Pose2d newPose) {
        return runOnce(() -> {});
    } // this doesn't apply to photonvision

    @Override
    public Command zeroHeading() {
        return runOnce(() -> {});
    }
}
