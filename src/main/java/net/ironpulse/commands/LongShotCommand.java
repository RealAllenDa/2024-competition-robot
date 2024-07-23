package net.ironpulse.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import net.ironpulse.Constants;
import net.ironpulse.Constants.HeadingController;
//import net.ironpulse.drivers.LimelightHelpers;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.FieldCentricHeading;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;
import net.ironpulse.utils.Utils;

import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.Constants.SwerveConstants.*;

import org.littletonrobotics.conduit.ConduitApi;

public class LongShotCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private final FieldCentricHeading drive = new FieldCentricHeading()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(0)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    //private final Translation2d targetTranslation = new Translation2d(2, 2); // modify this

    public LongShotCommand(
            ShooterSubsystem shooterSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            SwerveSubsystem swerveSubsystem,
            CommandXboxController driverController) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;
        drive.HeadingController.setPID(headingGains.kP, headingGains.kI, headingGains.kD);
        drive.HeadingController.enableContinuousInput(-180.0, 180.0);
        // drive.HeadingController.enableContinuousInput(0.0, 360.0);
        drive.HeadingController.setP(HeadingController.SNAP_HEADING_KP.get());
        drive.HeadingController.setD(HeadingController.SNAP_HEADING_KD.get());
        
    }
 
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(HeadingController.SNAP_HEADING_KP.hasChanged()) {
            debug("Changing Snap KP!");
            drive.HeadingController.setP(HeadingController.SNAP_HEADING_KP.get());
        }

        if(HeadingController.SNAP_HEADING_KD.hasChanged()) {
            debug("Changing Snap KD!");
            drive.HeadingController.setD(HeadingController.SNAP_HEADING_KD.get());
        }

        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
        double angle = 0;
        switch(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)){
            case Blue:
                angle = -1 * Constants.SwerveConstants.LongShotAngle.get();
                break;
            case Red:
                angle = Constants.SwerveConstants.LongShotAngle.get();
                break;
            default:
                break;
        }

        Rotation2d targetAngle = Rotation2d.fromDegrees(angle); // test this first
        // Translation2d currentTranslation =
        // swerveSubsystem.getPose().getTranslation();
        // Translation2d deltaTranslation = targetTranslation.minus(currentTranslation);.lk
        // Rotation2d targetAngle = deltaTranslation.getAngle();

        swerveSubsystem.applyRequest(() -> drive
                .withVelocityX(Utils.sign(-driverController.getLeftY())
                        * xLimiter.calculate(Math.abs(driverController.getLeftY()))
                        * maxSpeed.magnitude())
                .withVelocityY(
                        Utils.sign(-driverController.getLeftX()) * maxSpeed.magnitude()
                                * yLimiter.calculate(Math.abs(driverController.getLeftX())))
                .withCurrentAngle(swerveSubsystem.getState().Pose.getRotation().minus(
                    swerveSubsystem.getOffset()))
                .withTargetAngle(targetAngle))
                .execute();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }
}
