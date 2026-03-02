// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.kTurretConfiguration;
import static frc.robot.subsystems.shooter.ShooterConfiguration.kTurretOffset;

import java.util.Objects;
import java.util.Optional;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelConfig;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelIOReal;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelinputsAutoLogged;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.io.TurretIOReal;
//import frc.robot.subsystems.shooter.turret.io.TurretIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOReal;
import frc.robot.subsystems.vision.io.VisionIOSim;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.HubShiftUtil;
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
    // intake
    // shooter
    // climber
    private final Turret m_turret;
    private final FlywheelIO m_flywheel;
    private final FlywheelinputsAutoLogged flywheelInputs = new FlywheelinputsAutoLogged();
    private final LinearFilter filter = LinearFilter.movingAverage(10);

    public RobotContainer() {
        switch (RobotConstants.getBehavior()) {
            case COMPETITION -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOReal(13),
                    new SwerveModuleIOReal(kModuleConfigurations[0]),
                    new SwerveModuleIOReal(kModuleConfigurations[1]),
                    new SwerveModuleIOReal(kModuleConfigurations[2]),
                    new SwerveModuleIOReal(kModuleConfigurations[3])
                );

                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIOReal("FrontCamera", new Transform3d(
                        new Translation3d(0.0, Units.inchesToMeters(-2.0), 0.279),
                        new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-1.5)))
                    )
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

            case LOG_REPLAY, DEVELOPMENT -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {}
                );

                // m_vision = new VisionSubsystem(
                //     m_drivetrain.getPoseEstimator(),
                //     new VisionIO() {},
                //     new VisionIO() {}
                // );
                m_vision = new VisionSubsystem(
                    m_drivetrain.getPoseEstimator(),
                    new VisionIOReal("FrontCamera", new Transform3d(
                        new Translation3d(0.0, Units.inchesToMeters(-2.0), 0.279),
                        new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-1.5)))
                    )
                );
            }

            default -> throw new UnsupportedOperationException();
        }

        CommandScheduler.getInstance().registerSubsystem(
            m_vision,
            m_drivetrain//,
            //m_intake,
            //m_indexer,
            //m_shooter,
        );

        if (!RobotConstants.isCompetition()) {
            m_programmerController = new CommandXboxController(5);
        } else {
            m_programmerController = null;
        }

        m_turret = new Turret(new TurretIOReal(kTurretConfiguration));
        m_flywheel = new FlywheelIOReal(new FlywheelConfig(22, 23));

        configureBindings();
        configurePathPlanner();
        configureDashboard();
        SmartDashboard.putNumber("ShooterAngleDeg", 55.0);
        //m_turret.zeroTurretMotor();

        CommandScheduler.getInstance().schedule(
            Commands.waitUntil(() -> m_turret.getInputs().isTurretEncoderConnected)
            .andThen(Commands.runOnce(m_turret::syncTurretMotorEncoder))
        );
    }

    public final void periodic() {
        m_field.setRobotPose(m_drivetrain.getEstimatedPose());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Battery Voltage", RobotConstants.isReal() ?
            RobotController.getBatteryVoltage() : SimulatedBattery.getBatteryVoltage().in(Volts)
        );

        RobotVisualizer.updateComponents();
        m_turret.periodic();
        m_flywheel.updateInputs(flywheelInputs);
        Logger.processInputs("ShooterSubsystem/", flywheelInputs);
        // m_flywheel.setMotorVoltages(Volts.of(
        //     MathUtil.applyDeadband(
        //         -m_programmerController.getRightY(),
        //         0.25
        //     ) * 13.0
        // ));

        ShotCalculator.update(
            m_drivetrain.getEstimatedPose(),
            m_drivetrain.getChassisSpeeds(),
                ShooterUtils.getPolynomialVelocity(RadiansPerSecond.of(
                    filter.calculate(flywheelInputs.rightMotorVelocity.abs(RadiansPerSecond)))
                )
        );

        System.out.println(ShotCalculator.getLaunchAngle().in(Degrees));
    }

    private final void configureBindings() {
        m_drivetrain.setDefaultCommand(new ControllerDriveCommand(m_driverController, m_drivetrain));

        m_driverController.leftTrigger().whileTrue(TuningCommands.getCharacterizationRoutine(m_drivetrain));
        m_driverController.x().onTrue(Commands.runOnce(() -> m_drivetrain.zeroYaw()));
        //m_driverController.a().onTrue(Commands.runOnce(() -> m_turret.zeroTurretMotor()));
        //m_driverController.a().onTrue(Commands.runOnce(() -> m_turret.syncTurretMotor()).ignoringDisable(true));
        m_programmerController.rightTrigger().whileTrue(Commands.run(() -> {
            m_turret.setHoodAngle(Degrees.of(SmartDashboard.getNumber("ShooterAngleDeg", 55.0)));
        }));

        m_programmerController.b().onTrue(
            Commands.runOnce(() -> m_flywheel.setMotorVelocities(RadiansPerSecond.of(-325.0)))
        ).onFalse(Commands.runOnce(() -> m_flywheel.setMotorVelocities(RadiansPerSecond.of(0.0))));

        m_programmerController.a().onTrue(m_turret.zeroHoodRoutine());
        m_programmerController.leftTrigger().whileTrue(Commands.run(() -> {
        m_turret.setHoodMotorVoltage(Volts.of(
            MathUtil.applyDeadband(
                -m_programmerController.getLeftY(),
                0.25
            ) * 5.0
        ));
        }));

        if (!RobotConstants.isCompetition()) {
            m_programmerController.rightTrigger().whileTrue(new ControllerDriveCommand(m_programmerController, m_drivetrain));
            m_programmerController.a().whileTrue(TuningCommands.getWheelRadiusCommand(m_drivetrain));
            //m_programmerController.b().whileFalse(TuningCommands.getCharacterizationRoutine(m_drivetrain));
        }

        if (RobotConstants.isSimulated()) {
            m_driverController.rightTrigger().whileTrue(Commands.repeatingSequence(
                Commands.runOnce(() -> {
                    ShotCalculator.update(
                        m_drivetrain.getEstimatedPose(),
                        m_drivetrain.getChassisSpeeds(),
                        MetersPerSecond.of(8.0)
                    );

                    Logger.recordOutput("ShotTarget", ShotCalculator.getTargetFieldRelative());

                    SimulatedArena.getInstance().addGamePieceProjectile(
                        new RebuiltFuelOnFly(
                            m_drivetrain.getSimulationPose().getTranslation().plus(kTurretOffset.rotateBy(m_drivetrain.getSimulationPose().getRotation())),
                            Translation2d.kZero,
                            m_drivetrain.getFieldChassisSpeeds(true),
                            ShotCalculator.getTargetTurretRelative().getAngle(),
                            Meters.of(0.762),
                            MetersPerSecond.of(8.0),
                            ShotCalculator.getLaunchAngle()
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
