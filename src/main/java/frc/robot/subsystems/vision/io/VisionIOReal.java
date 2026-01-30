package frc.robot.subsystems.vision.io;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import frc.robot.utils.FieldUtils;

public class VisionIOReal implements VisionIO {
    protected final PhotonCamera m_camera;
    protected final PhotonPoseEstimator m_poseEstimator;

    public VisionIOReal(String cameraName, Transform3d cameraTransform) {
        m_camera = new PhotonCamera(cameraName);
        m_poseEstimator = new PhotonPoseEstimator(FieldUtils.getAprilTagLayout(), cameraTransform);
    }

    @Override
    public void updateInputs(VisionInputs loggableInputs, Pose2d estimatedPose) {
        List<PhotonPipelineResult> cameraResults = m_camera.getAllUnreadResults();

        loggableInputs.isConnected = m_camera.isConnected();
        loggableInputs.poseEstimations = getPoseEstimations(cameraResults);
        loggableInputs.aprilTagDetections = getAprilTagDetections(cameraResults);
    }

    private final LoggablePoseEstimation[] getPoseEstimations(List<PhotonPipelineResult> cameraResults) {
        List<LoggablePoseEstimation> poseEstimations = new ArrayList<>();

        for (PhotonPipelineResult cameraResult : cameraResults) {
            var poseEstimationResult = m_poseEstimator.estimateCoprocMultiTagPose(cameraResult);

            if (poseEstimationResult.isPresent()) {
                poseEstimations.add(new LoggablePoseEstimation(
                    poseEstimationResult.get().estimatedPose,
                    poseEstimationResult.get().timestampSeconds
                ));
            }
        }

        return poseEstimations.toArray(LoggablePoseEstimation[]::new);
    }

    private final LoggableAprilTagDetection[][] getAprilTagDetections(List<PhotonPipelineResult> cameraResults) {
        List<LoggableAprilTagDetection[]> aprilTagDetections = new ArrayList<>();

        for (PhotonPipelineResult cameraResult : cameraResults) {
            var aprilTags = cameraResult.getTargets();

            aprilTagDetections.add(aprilTags.stream()
                .map(aprilTag -> new LoggableAprilTagDetection(
                    aprilTag.fiducialId,
                    aprilTag.area,
                    aprilTag.bestCameraToTarget.getTranslation().getNorm()
                ))
                .toArray(LoggableAprilTagDetection[]::new)
            );
        }

        return aprilTagDetections.toArray(LoggableAprilTagDetection[][]::new);
    }
}
