// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;
import static frc.robot.subsystems.intake.IntakeConfiguration.kIntakeHardware;
import static frc.robot.subsystems.shooter.ShooterConfiguration.FlywheelConfiguration.*;
import static frc.robot.subsystems.shooter.ShooterConfiguration.TurretConfiguration.*;

import java.util.Objects;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.SimCameraProperties;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
import frc.robot.commands.ShooterControlCommand;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOReal;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOSim;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOReal;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOSim;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.io.IndexerIOReal;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.io.IntakeIOReal;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.io.FlywheelIOReal;
import frc.robot.subsystems.shooter.kicker.io.KickerIO;
import frc.robot.subsystems.shooter.kicker.io.KickerIOReal;
import frc.robot.subsystems.shooter.turret.io.TurretIOReal;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.VisionIOReal;
import frc.robot.subsystems.vision.io.VisionIOSim;
import frc.robot.utils.RobotVisualizer;
import frc.robot.utils.ShooterUtils;

public final class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_programmerController = new CommandXboxController(5);

    private final Field2d m_field = new Field2d();
    private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto");
    private final Alert m_autoChooserAlert = new Alert("No autonomous selected.", AlertType.kWarning);

    private final VisionSubsystem m_vision;
    private final DrivetrainSubsystem m_drivetrain;
    private final IntakeSubsystem m_intake;
    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;

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

                m_intake = null;
                m_shooter = null;
                m_indexer = null;
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

                m_intake = null;
                m_shooter = null;
                m_indexer = null;

                SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
                SimulatedArena.getInstance().addDriveTrainSimulation(drivetrainSimulation);
            }

            case LOG_REPLAY, DEVELOPMENT -> {
                m_drivetrain = new DrivetrainSubsystem(
                    new GyroIOReal(13),
                    new SwerveModuleIOReal(kModuleConfigurations[0]),
                    new SwerveModuleIOReal(kModuleConfigurations[1]),
                    new SwerveModuleIOReal(kModuleConfigurations[2]),
                    new SwerveModuleIOReal(kModuleConfigurations[3])
                );

                m_vision = null;

                // m_vision = new VisionSubsystem(
                //     m_drivetrain.getPoseEstimator(),
                //     new VisionIOReal("BackLeftCamera", new Transform3d(
                //         new Translation3d(0.0, 0.0, 0.0),
                //         new Rotation3d(0.0, 0.0, 0.0)
                //     )),

                //     new VisionIOReal("BackRightCamera", new Transform3d(
                //         new Translation3d(0.0, 0.0, 0.0),
                //         new Rotation3d(0.0, 0.0, 0.0)
                //     ))
                //     // new VisionIOReal("FrontCamera", new Transform3d(
                //     //     new Translation3d(0.0, Units.inchesToMeters(-2.0), 0.279),
                //     //     new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-1.5)))
                //     // )
                // );

                m_intake = new IntakeSubsystem(
                    new IntakeIOReal(kIntakeHardware)
                );

                m_indexer = new IndexerSubsystem(new IndexerIOReal(34));

                m_shooter = new ShooterSubsystem(
                    new KickerIOReal(37),
                    new TurretIOReal(kTurretHardware),
                    new FlywheelIOReal(kFlywheelHardware)
                );
            }

            default -> throw new UnsupportedOperationException();
        }

        // TODO: Look into implementing subsystems like 6328's 2026.
        CommandScheduler.getInstance().registerSubsystem(
            m_vision,
            m_drivetrain,
            m_intake,
            m_indexer,
            m_shooter
        );

        configureBindings();
        configurePathPlanner();
        configureDashboard();

        // SmartDashboard.putNumber("TargetShooterRad", 0.0);

        // After every power-cycle, we re-zero on enable.
        // CommandScheduler.getInstance().schedule(m_shooter.getZeroRoutine().ignoringDisable(true));
    }

    public final void periodic() {
        m_field.setRobotPose(m_drivetrain.getEstimatedPose());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Battery Voltage", RobotConstants.isReal() ?
            RobotController.getBatteryVoltage() : SimulatedBattery.getBatteryVoltage().in(Volts)
        );

        // ShotCalculator.update(
        //     m_drivetrain.getEstimatedPose(),
        //     m_drivetrain.getChassisSpeeds()
        // );

        // if (m_programmerController.b().getAsBoolean()) {
        //     m_shooter.setFlywheelVelocity(
        //         RadiansPerSecond.of(SmartDashboard.getNumber("TargetShooterRad", 0.0))
        //     );
        // } else {
        //     m_shooter.stopFlywheel();
        // }

        // // if (m_programmerController.b().getAsBoolean()) {
        // //     m_shooter.setFlywheelVelocity(ShooterUtils.getPolynomialVelocityRoot(
        // //         ShotCalculator.getFuelVelocity()
        // //     ));
        // // } else {
        // //     m_shooter.stopFlywheel();
        // // }

        // // if (m_programmerController.a().getAsBoolean()) {
        // //     m_shooter.setHoodAngle(ShotCalculator.getLaunchAngle());
        // // }

        // if (m_programmerController.rightTrigger().getAsBoolean()) {
        //     m_shooter.setHoodVoltage(Volts.of(MathUtil.applyDeadband(
        //         -m_programmerController.getRightX(),
        //         0.25
        //     ) * 3.0));
        // } else {
        //     m_shooter.setHoodVoltage(Volts.zero());
        // }

        if (m_programmerController.leftBumper().getAsBoolean()) {
            m_shooter.setFlywheelVoltage(Volts.of(8.0));
        } else {
            m_shooter.setFlywheelVoltage(Volts.of(0.0));
        }

        if (m_programmerController.leftTrigger().getAsBoolean()) {
            m_intake.startIntake();
            m_indexer.start();
            m_shooter.startKicker();
        } else {
            m_intake.stopIntake();
            m_indexer.stop();
            m_shooter.stopKicker();
        }

        // if (m_programmerController.a().getAsBoolean()) {
        //     // TODO: Should we require the individual components of the shooter instead of it as a subsystem?
        //     // We could have a commmand to run the kicker that requires the kicker, have a hood angle command, etc.
        //     Commands.runOnce(() -> m_shooter.setHoodAngle(ShotCalculator.getLaunchAngle()), m_shooter);
        // } else {
        //     Commands.runOnce(m_shooter::stopHood, m_shooter);
        // }

        RobotVisualizer.updateComponents();
        // System.out.println("Angle: " + ShotCalculator.getLaunchAngle().in(Degrees) + ", Speed: " + ShotCalculator.getFuelVelocity());
    }

    private final void configureBindings() {
        m_drivetrain.setDefaultCommand(new ControllerDriveCommand(m_driverController, m_drivetrain));
        // m_shooter.setDefaultCommand(new ShooterControlCommand(m_shooter, m_drivetrain::getEstimatedPose, m_drivetrain::getChassisSpeeds));
        // m_programmerController.x().onTrue(m_shooter.getZeroRoutine());

        if (!RobotConstants.isCompetition()) {
            // m_programmerController.rightTrigger().whileTrue(new ControllerDriveCommand(m_programmerController, m_drivetrain));
            // m_programmerController.leftBumper().whileTrue(TuningCommands.getWheelRadiusCommand(m_drivetrain));
            // m_programmerController.rightBumper().whileTrue(TuningCommands.getCharacterizationRoutine(m_drivetrain));
            // m_programmerController.x().onTrue(Commands.runOnce(m_drivetrain::zeroYaw));
            // m_programmerController.a().onTrue(Commands.runOnce(m_intake::toggleIntakeExtended));
            // m_programmerController.b().onTrue(Commands.runOnce(m_intake::startIntake)).onFalse(Commands.runOnce(m_intake::stopIntake));
        }

        if (RobotConstants.isSimulated()) {}
    }

    private final void configurePathPlanner() {
        PathPlannerLogging.setLogTargetPoseCallback(
            pose -> m_field.getObject("target pose").setPose(pose)
        );

        PathPlannerLogging.setLogActivePathCallback(
            poses -> m_field.getObject("path").setPoses(poses)
        );

        m_autoChooser.addDefaultOption("None", Commands.none());
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
