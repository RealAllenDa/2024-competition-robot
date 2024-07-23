package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indexer.IndexerSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Degrees;
import static net.ironpulse.Constants.ShooterConstants.farShootVoltage;

public class HighShootCommand extends ParallelCommandGroup {
    public HighShootCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            SwerveSubsystem swerveSubsystem,
            CommandXboxController drivercController
    ) {
        addCommands(
                new ParallelAimingCommand(shooterSubsystem, Degrees.of(13)),
                new PreShootWithoutAimingCommand(shooterSubsystem, farShootVoltage),
                new LongShotCommand(shooterSubsystem, indicatorSubsystem, swerveSubsystem, drivercController),
                Commands.sequence(
                        new WaitCommand(0.4),
                        new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)
                )

        );
    }
}
