package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.Optional;

/** Implements a Vision Subsystem for QuestNav */
public class QuestNavSubsystem extends VisionSubsystem {

    public static class QuestNavConstants {
        public static final Matrix<N3, N1> QUESTNAV_CAM_VISION_TRUST = VecBuilder.fill(0.02, 0.02, 0.035);

        public static final Transform2d QUESTNAV_CAM_RELATIVE_TO_ROBOT =
                new Transform2d(new Translation2d(Units.inchesToMeters(12.0), 0), new Rotation2d(0));
    }

    public QuestNav quest = new QuestNav();

    public String getName() {
        return "quest nav";
    }

    @Override
    public Command zeroHeading() {
        return runOnce(() -> {});
    }

    protected Optional<VisionData> getVisionResult() {
        quest.commandPeriodic();

        Optional<PoseFrame> lastEstimatedPose = Optional.empty();

        for (var result : quest.getAllUnreadPoseFrames()) {
            lastEstimatedPose = Optional.of(result);
        } // if getAllUnreadPoseFrames() is empty then lastEstimatedPose will be Optional.empty()
        // this also accounts for results that have data but are surrounded by results without data

        if (lastEstimatedPose.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(new VisionData(
                lastEstimatedPose.get().questPose(),
                lastEstimatedPose.get().dataTimestamp(),
                QuestNavConstants.QUESTNAV_CAM_VISION_TRUST));
    }

    /**
     * Method to determine if QuestNav is ready
     *
     * @return True if the Quest is ready and tracking, false otherwise
     */
    public boolean isReady() {
        return (quest.isConnected() && quest.isTracking());
    }

    /**
     * Resets the Quest headset position to the specified pose.
     *
     * @param pose The desired pose as as a {@link Pose2d} Object
     */
    @Override
    public Command resetPose(Pose2d pose) {
        return runOnce(() -> {
            Translation2d robotTranslation =
                    pose.getTranslation().plus(QuestNavConstants.QUESTNAV_CAM_RELATIVE_TO_ROBOT.getTranslation());
            Pose2d questPose = new Pose2d(
                    robotTranslation,
                    pose.getRotation().plus(QuestNavConstants.QUESTNAV_CAM_RELATIVE_TO_ROBOT.getRotation()));
            quest.setPose(questPose);
        });
    }
}
