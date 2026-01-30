package frc.robot.subsystems.vision.io;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.RobotBehavior;
import frc.robot.utils.FieldUtils;

public class VisionIOSim extends VisionIOReal {
    private final PhotonCameraSim m_cameraSimulation;
    private final Supplier<Pose2d> m_simulationPoseSupplier;
    private static final VisionSystemSim m_visionSimulation;

    static {
        if (RobotConstants.BEHAVIOR == RobotBehavior.SIMULATED) {
            m_visionSimulation = new VisionSystemSim("PhotonVision");
            m_visionSimulation.addAprilTags(FieldUtils.getAprilTagLayout());
        } else {
            m_visionSimulation = null;
        }
    }

    public VisionIOSim(
        String cameraName,
        Transform3d cameraTransform,
        SimCameraProperties cameraProperties,
        Supplier<Pose2d> simulationPoseSupplier
    ) {
        super(cameraName, cameraTransform);
        m_cameraSimulation = new PhotonCameraSim(m_camera, cameraProperties);
        m_simulationPoseSupplier = simulationPoseSupplier;

        m_visionSimulation.addCamera(m_cameraSimulation, cameraTransform);
    }

    @Override
    public void updateInputs(VisionInputs loggableInputs, Pose2d estimatedPose) {
        m_visionSimulation.update(m_simulationPoseSupplier.get());
        super.updateInputs(loggableInputs, estimatedPose);
    }
}
