package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.turret.io.TurretIO;

public final class ShooterSubsystem implements Subsystem {
    public ShooterSubsystem() {}

    public static Command zeroHoodRoutine() {
        return Commands.none();
    }
}
