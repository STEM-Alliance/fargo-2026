// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.drivetrain.DrivetrainConfiguration.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ControllerDriveCommand;
import frc.robot.commands.TuningCommands;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOReal;
import frc.robot.subsystems.drivetrain.gyro.io.GyroIOSim;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOReal;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIOSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.VisionIOReal;
import frc.robot.subsystems.vision.io.VisionIOSim;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.ShooterUtils;

public final class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final CommandXboxController m_programmerController = new CommandXboxController(5);

    private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto");
    private final Alert m_autoChooserAlert = new Alert("No autonomous selected.", AlertType.kWarning);

    private final DrivetrainSubsystem m_drivetrain;
    private final VisionSubsystem m_vision;

    public RobotContainer() {
        switch (RobotConstants.BEHAVIOR) {
            case REAL -> {
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

            case SIMULATED -> {
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

            default -> throw new UnsupportedOperationException();
        }

        CommandScheduler.getInstance().registerSubsystem(m_drivetrain, m_vision);

        configureBindings();
        configureAutoChooser();
        configureAlerts();
    }

    private final void configureBindings() {
        // m_driverController.x().onTrue(Commands.runOnce(() -> {m_drivetrain.zeroYaw();}));
        // m_driverController.rightTrigger().whileTrue(new AutoAimCommand(m_driverController, m_drivetrain));

        // m_programmerController.a().whileTrue(TuningCommands.getWheelRadiusCommand(m_drivetrain));
        // m_programmerController.b().whileTrue(TuningCommands.getCharacterizationRoutine(m_drivetrain));
        // m_programmerController.rightTrigger().whileTrue(new ControllerDriveCommand(m_programmerController, m_drivetrain));

        // m_drivetrain.setDefaultCommand(new ControllerDriveCommand(m_driverController, m_drivetrain));
        // m_driverController.a().whileTrue(Commands.repeatingSequence(new InstantCommand(() -> {
        //     // var hubPose = FieldUtils.getAllianceHub();
        //     // var robotPose = m_drivetrain.getEstimatedPose();
        //     // var hubRelative = hubPose.minus(robotPose.getTranslation());
        //     // var speedTemp = m_drivetrain.getChassisSpeeds();
        //     // speedTemp = ChassisSpeeds.fromRobotRelativeSpeeds(speedTemp, hubRelative.getAngle());
        //     // var shooterAngle = ShooterUtils.getLaunchAngle(Meters.of(hubRelative.getNorm()), MetersPerSecond.of(8.0 + speedTemp.vxMetersPerSecond));
        //     var hubPose = FieldUtils.getAllianceHub();
        //     var robotPose = m_drivetrain.getEstimatedPose();
        //     var hubRelative = hubPose.minus(robotPose.getTranslation());
        //     var speedTemp = m_drivetrain.getChassisSpeeds();

        //     // Project robot velocity onto vector toward the hub
        //     double hubDistance = hubRelative.getNorm();
        //     double robotVelAlongHub = 0.0;
        //     if (hubDistance > 1e-6) {
        //         var hubUnit = hubRelative.div(hubDistance); // unit vector along hub
        //         var robotVel = new Translation2d(speedTemp.vxMetersPerSecond, speedTemp.vyMetersPerSecond);
        //         robotVelAlongHub = robotVel.dot(hubUnit);
        //     }

        //     // Use projected velocity instead of just speedTemp.vxMetersPerSecond
        //     var shooterAngle = ShooterUtils.getLaunchAngle(
        //         Meters.of(hubDistance),
        //         MetersPerSecond.of(8.0 + robotVelAlongHub)
        //     );

        //     SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
        //         m_drivetrain.getSimulationPose().getTranslation(),
        //         new Translation2d(),
        //         m_drivetrain.getChassisSpeeds(),
        //         m_drivetrain.getSimulationPose().getRotation(),
        //         Meters.of(0.762),
        //         MetersPerSecond.of(8.0),
        //         shooterAngle
        //     ));
        // }), new WaitCommand(0.15)));
    }

    private final void configureAutoChooser() {
        m_autoChooser.addDefaultOption("None", new InstantCommand());
    }

    private final void configureAlerts() {
        m_autoChooserAlert.set(true);
        m_autoChooser.getSendableChooser().onChange(key -> {
            m_autoChooserAlert.set(key.equals("None"));
        });
    }

    public final Command getAutonomousCommand() {
        return m_autoChooser.get();
    }
}
