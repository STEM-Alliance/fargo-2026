// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.kTurretConfiguration;

import java.util.Objects;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.SimCameraProperties;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.ControllerDriveCommand;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOReal;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOSim;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOReal;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.io.TurretIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOReal;
import frc.robot.subsystems.vision.io.VisionIOSim;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.RobotVisualizer;
import frc.robot.utils.ShooterUtils;

public final class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_programmerController;

    private final Field2d m_field = new Field2d();
    private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto");
    private final Alert m_autoChooserAlert = new Alert("No autonomous selected.", AlertType.kWarning);

    private final DrivetrainSubsystem m_drivetrain;
    private final VisionSubsystem m_vision;

    private final Turret m_turret;

    public RobotContainer() {
        switch (RobotConstants.getBehavior()) {
            case COMPETITION, DEVELOPMENT -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOReal(13),
                    new SwerveModuleIOReal(kModuleConfigurations[0]),
                    new SwerveModuleIOReal(kModuleConfigurations[1]),
                    new SwerveModuleIOReal(kModuleConfigurations[2]),
                    new SwerveModuleIOReal(kModuleConfigurations[3])
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIOReal("FrontCAM", new Transform3d(
                        new Translation3d(0.0254, 0.0, 0.279),
                        new Rotation3d(0.0, 0.0, 0.0)
                    )),

                    new VisionIOReal("BackCAM", new Transform3d(
                        new Translation3d(-0.0254, 0.0, 0.279),
                        new Rotation3d(0.0, 0.0, Math.PI)
                    ))
                );
            }

            case SIMULATION -> {
                var drivetrainSimulation = new SwerveDriveSimulation(kSimulationConfig, kSimulationStartingPose);

                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOSim(13, drivetrainSimulation.getGyroSimulation()),
                    new SwerveModuleIOSim(kModuleConfigurations[0], drivetrainSimulation.getModules()[0]),
                    new SwerveModuleIOSim(kModuleConfigurations[1], drivetrainSimulation.getModules()[1]),
                    new SwerveModuleIOSim(kModuleConfigurations[2], drivetrainSimulation.getModules()[2]),
                    new SwerveModuleIOSim(kModuleConfigurations[3], drivetrainSimulation.getModules()[3]),
                    drivetrainSimulation
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIOSim(
                        "PerfectCAM",
                        new Transform3d(),
                        new SimCameraProperties(),
                        m_drivetrain::getSimulationPose
                    )
                );

                SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
                SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSimulation);
            }

            case LOG_REPLAY -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {}
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIO() {},
                    new VisionIO() {}
                );
            }

            default -> throw new UnsupportedOperationException();
        }

        CommandScheduler.getInstance().registerSubsystem(m_drivetrain, m_vision);

        if (!RobotConstants.isCompetition()) {
            m_programmerController = new CommandXboxController(5);
        } else {
            m_programmerController = null;
        }

        configureBindings();
        configurePathPlanner();
        configureDashboard();
        m_turret = null;//new Turret(new TurretIOSim(kTurretConfiguration));
        SmartDashboard.putNumber("DesiredAzimuth", 0.0);
    }

    public final void periodic() {
        m_field.setRobotPose(m_drivetrain.getEstimatedPose());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Battery Voltage", RobotConstants.isReal() ?
            RobotController.getBatteryVoltage() : SimulatedBattery.getBatteryVoltage().in(Volts)
        );

        RobotVisualizer.updateComponents();
        // m_turret.periodic();
        // m_turret.setTurretAzimuth(
        //     Radians.of(
        //         FieldUtils.getAllianceHub().minus(m_drivetrain.getEstimatedPose().getTranslation())
        //         .getAngle().plus(m_drivetrain.getEstimatedPose().getRotation()).getRadians()
        //     )
        // );

        //Logger.recordOutput("FacingDirection", new Translation2d(2.5, 0.0).rot);
        //m_turret.setTurretAzimuth(Degrees.of(SmartDashboard.getNumber("DesiredAzimuth", 0.0)));
    }

    private final void configureBindings() {
        m_drivetrain.setDefaultCommand(new ControllerDriveCommand(m_driverController, m_drivetrain));

        m_driverController.leftTrigger().whileTrue(TuningCommands.getCharacterizationRoutine(m_drivetrain));
        m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrain.zeroYaw()));
        //m_driverController.a().onTrue(Commands.runOnce(() -> m_turret.zeroTurretMotor()));

        if (!RobotConstants.isCompetition()) {
            m_programmerController.rightTrigger().whileTrue(new ControllerDriveCommand(m_programmerController, m_drivetrain));
            m_programmerController.a().whileTrue(TuningCommands.getWheelRadiusCommand(m_drivetrain));
            m_programmerController.b().whileFalse(TuningCommands.getCharacterizationRoutine(m_drivetrain));
        }

        if (RobotConstants.isSimulated()) {
            m_driverController.rightTrigger().whileTrue(Commands.repeatingSequence(
                Commands.runOnce(() -> {
                    Translation2d target;

                    if (FieldUtils.inFriendlyAllianceZone(m_drivetrain.getEstimatedPose())) {
                        target = FieldUtils.getAllianceHub();
                    } else {
                        target = ShooterUtils.getPassingTarget(
                            m_drivetrain.getEstimatedPose(),
                            Meters.of(0.75),
                            Meters.of(1.25)
                        );
                    }

                    Logger.recordOutput("DirectTarget", new Pose2d(target.minus(m_drivetrain.getEstimatedPose().getTranslation()), Rotation2d.kZero));

                    target = ShooterUtils.getLeadedTranslation(
                        m_drivetrain.getEstimatedPose(),
                        target,
                        MetersPerSecond.of(8.0),
                        m_drivetrain.getChassisSpeeds()
                    );

                    Logger.recordOutput("LeadedTarget", new Pose2d(target, Rotation2d.kZero));

                    SimulatedArena.getInstance().addGamePieceProjectile(
                        new RebuiltFuelOnFly(
                            m_drivetrain.getSimulationPose().getTranslation(),
                            Translation2d.kZero,
                            m_drivetrain.getFieldChassisSpeeds(true),
                            target.getAngle(),
                            Meters.of(0.762),
                            MetersPerSecond.of(8.0),
                            FieldUtils.inFriendlyAllianceZone(m_drivetrain.getEstimatedPose()) ? ShooterUtils.getQuadraticAngles(Meters.of(target.getNorm()), Meters.of(1.52), MetersPerSecond.of(8.0)).getSecond(): Degrees.of(22.5)
                        )
                    );
                }),

                Commands.waitSeconds(0.2)
            ));
        }
    }

    private final void configurePathPlanner() {
        PathPlannerLogging.setLogTargetPoseCallback(
            pose -> m_field.getObject("target pose").setPose(pose)
        );

        PathPlannerLogging.setLogActivePathCallback(
            poses -> m_field.getObject("path").setPoses(poses)
        );

        // NamedCommands.registerCommand(null, Commands.none());

        m_autoChooser.addDefaultOption("None", Commands.none());
        // m_autoChooser.addOption(null, new PathPlannerAuto());
    }

    private final void configureDashboard() {
        m_autoChooserAlert.set(true);
        m_autoChooser.getSendableChooser().onChange(
            key ->  m_autoChooserAlert.set(key.equals("None"))
        );

        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    public final Command getAutonomousCommand() {
        return Objects.requireNonNullElse(m_autoChooser.get(), Commands.none());
    }
}
