package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionInputsAutoLogged;
import frc.robot.subsystems.vision.io.VisionIO.LoggableAprilTagDetection;

public final class VisionSubsystem implements Subsystem {
    private final PoseEstimator<?> m_poseEstimator;
    private final List<Pair<VisionIO, VisionInputsAutoLogged>> m_cameras = new ArrayList<>();

    public VisionSubsystem(PoseEstimator<?> poseEstimator, VisionIO... visionIOs) {
        m_poseEstimator = poseEstimator;

        for (VisionIO visionIO : visionIOs) {
            m_cameras.add(Pair.of(visionIO, new VisionInputsAutoLogged()));
        }
    }

    @Override
    public final void periodic() {
        for (int i = 0; i < m_cameras.size(); i++) {
            var visionIO = m_cameras.get(i).getFirst();
            var visionInputs = m_cameras.get(i).getSecond();

            visionIO.updateInputs(visionInputs, m_poseEstimator.getEstimatedPosition());
            Logger.processInputs("VisionSubsystem/Cameras/Camera" + i, visionInputs);

            for (int j = 0; j < visionInputs.poseEstimations.length; j++) {
                var poseEstimation = visionInputs.poseEstimations[j];
                var aprilTagDetections = visionInputs.aprilTagDetections[j];

                m_poseEstimator.addVisionMeasurement(
                    poseEstimation.pose().toPose2d(),
                    poseEstimation.timestamp(),
                    getPoseStdDevs(aprilTagDetections)
                );
            }
        }
    }

    private static final Matrix<N3, N1> getPoseStdDevs(LoggableAprilTagDetection[] aprilTags) {
        double totalArea = 0.0;

        for (var aprilTag : aprilTags) {
            totalArea += aprilTag.area();
        }

        if (aprilTags.length <= 1) {
            if ((aprilTags.length == 0) || (aprilTags[0].distance() > kMaxSingleTagDistance.in(Meters))) {
                return kIgnoredStdDevs;
            } else {
                return kSingleTagStdDevs.div(totalArea);
            }
        }

        return kMultiTagStdDevs.div(totalArea / (1.0 + Math.log(aprilTags.length) / Math.log(2.0)));
    }
}
