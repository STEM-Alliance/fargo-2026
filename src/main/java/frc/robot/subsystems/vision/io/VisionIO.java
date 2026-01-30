package frc.robot.subsystems.vision.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * The generic vision IO interface (with support for AdvantageKit).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface VisionIO {
    @AutoLog
    public static abstract class VisionInputs {
        public boolean isConnected = false;
        public LoggablePoseEstimation[] poseEstimations = new LoggablePoseEstimation[0];
        public LoggableAprilTagDetection[][] aprilTagDetections = new LoggableAprilTagDetection[0][];
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
     * @param estimatedPose The current pose estimation (for some pose strategies).
    */
    public default void updateInputs(VisionInputs loggableInputs, Pose2d estimatedPose) {}

    public static final record LoggablePoseEstimation(Pose3d pose, double timestamp) {}
    public static final record LoggableAprilTagDetection(int id, double area, double distance) {}
}
