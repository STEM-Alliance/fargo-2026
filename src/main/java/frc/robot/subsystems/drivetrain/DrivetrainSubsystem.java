package frc.robot.subsystems.drivetrain;

import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.gyro.Gyro;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIO;
import frc.robot.subsystems.drivetrain.swervemodule.SwerveModule;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIO;
import frc.robot.utils.FieldUtils;

public final class DrivetrainSubsystem implements Subsystem {
    private final Gyro m_gyro;
    private final SwerveModule[] m_swerveModules;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        kModuleConfigurations[0].moduleTranslation(),
        kModuleConfigurations[1].moduleTranslation(),
        kModuleConfigurations[2].moduleTranslation(),
        kModuleConfigurations[3].moduleTranslation()
    );

    private final SwerveDriveSimulation m_simulation;

    public DrivetrainSubsystem(
        GyroIO gyroIO,
        SwerveModuleIO flSwerveModuleIO,
        SwerveModuleIO frSwerveModuleIO,
        SwerveModuleIO blSwerveModuleIO,
        SwerveModuleIO brSwerveModuleIO,
        SwerveDriveSimulation... simulation
    ) {
        m_gyro = new Gyro(gyroIO);
        m_swerveModules = new SwerveModule[] {
            new SwerveModule("FL", flSwerveModuleIO),
            new SwerveModule("FR", frSwerveModuleIO),
            new SwerveModule("BL", blSwerveModuleIO),
            new SwerveModule("BR", brSwerveModuleIO)
        };

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            m_gyro.getAngleAsRotation(),
            getSwerveModulePositions(),
            Pose2d.kZero
        );

        if (simulation.length != 0) {
            m_simulation = simulation[0];
        } else {
            m_simulation = null;
        }
    
        RobotConfig robotConfig;

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // TODO: Add better exception handling
            robotConfig = null;
        }

        AutoBuilder.configure(
            this::getEstimatedPose,
            this::resetEstimatedPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> drive(speeds, false, false, feedforwards.accelerationsMPSSq()),
            kPathplannerController,
            robotConfig,
            () -> {return !FieldUtils.isBlueAlliance();},
            this

        );
    }

    @Override
    public final void periodic() {
        m_gyro.periodic();

        for (var swerveModule : m_swerveModules) {
            swerveModule.periodic();
        }

        m_poseEstimator.update(m_gyro.getAngleAsRotation(), getSwerveModulePositions());
        Logger.recordOutput("DrivetrainSubsystem/ChassisSpeeds", getChassisSpeeds());
        Logger.recordOutput("DrivetrainSubsystem/CurrentStates", getSwerveModuleStates());
        Logger.recordOutput("DrivetrainSubsystem/EstimatedPose", getEstimatedPose());
    }

    @Override
    public final void simulationPeriodic() {
        Logger.recordOutput("DrivetrainSubsystem/SimulationPose", getSimulationPose());
    }

    public final void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean blueRelative, double... gain) {
        Rotation2d estimatedRotation = getEstimatedPose().getRotation();

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds,
                blueRelative ? estimatedRotation : estimatedRotation.plus(Rotation2d.kPi)
            );
        }

        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Robot.defaultPeriodSecs);
        SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 4.0);
        Logger.recordOutput("DrivetrainSubsystem/DesiredStates", desiredStates);

        for (int i = 0; i < m_swerveModules.length; i++) {
            if (gain.length > 0) {
                m_swerveModules[i].setDriveFFAccel(gain[i]);
            } else {
                m_swerveModules[i].setDriveFFAccel(0.0);
            }

            m_swerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    public final void zeroYaw() {
        m_gyro.zero();
    }

    public final void setSwerveModulesWheelVelocity(LinearVelocity velocity) {
        for (var swerveModule : m_swerveModules) swerveModule.setWheelVelocity(velocity);
    }

    public final void setSwerveModulesWheelAzimuth(Angle azimuth) {
        for (var swerveModule : m_swerveModules) swerveModule.setWheelAzimuth(azimuth);
    }

    public final void setSwerveModulesDriveMotorVoltage(Voltage voltage) {
        for (var swerveModule : m_swerveModules) swerveModule.setDriveMotorVoltage(voltage);
    }

    public final void setSwerveModulesAzimuthMotorVoltage(Voltage voltage) {
        for (var swerveModule : m_swerveModules) swerveModule.setAzimuthMotorVoltage(voltage);
    }

    public final void resetEstimatedPose(Pose2d resetPose) {
        m_poseEstimator.resetPosition(
            m_gyro.getAngleAsRotation(),
            getSwerveModulePositions(),
            resetPose
        );
    }

    public final SwerveModule[] getSwerveModules() {
        return m_swerveModules;
    }

    public final SwerveDrivePoseEstimator getPoseEstimator() {
        return m_poseEstimator;
    }

    public final Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public final Pose2d getSimulationPose() {
        return m_simulation.getSimulatedDriveTrainPose();
    }

    public final ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public final ChassisSpeeds getFieldChassisSpeeds(boolean useSimulationPose) {
        Pose2d fieldPose = useSimulationPose ? getSimulationPose() : getEstimatedPose();
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), fieldPose.getRotation());
    }

    public final SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            m_swerveModules[0].getPosition(),
            m_swerveModules[1].getPosition(),
            m_swerveModules[2].getPosition(),
            m_swerveModules[3].getPosition(),
        };
    }

    public final SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            m_swerveModules[0].getState(),
            m_swerveModules[1].getState(),
            m_swerveModules[2].getState(),
            m_swerveModules[3].getState(),
        };
    }
}
