package net.ironpulse.commands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import net.ironpulse.Constants;
import net.ironpulse.Constants.ShooterConstants;
import net.ironpulse.drivers.LimelightHelpers;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterIO;
import net.ironpulse.subsystems.shooter.ShooterIOTalonFX;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.FieldCentricTargetHeading;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;
import net.ironpulse.utils.ShootingParameters;
import net.ironpulse.utils.ShootingParametersTable;
import net.ironpulse.utils.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.Constants.SwerveConstants.*;
import static net.ironpulse.Constants.HeadingController;

public class SpeakerAimingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private Measure<Angle> defaultAngle = Degrees.of(20);
    private final FieldCentricTargetHeading drive = new FieldCentricTargetHeading()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(0)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

    public SpeakerAimingCommand(
            ShooterSubsystem shooterSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            SwerveSubsystem swerveSubsystem,
            CommandXboxController driverController
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;
        drive.HeadingController.setPID(headingGains.kP, headingGains.kI, headingGains.kD);
        drive.HeadingController.setP(HeadingController.HEADING_KP.get());
        drive.HeadingController.setD(HeadingController.HEADING_KD.get());
        //drive.HeadingController.s;
        //drive.HeadingController.set
    }
    

    @Override
    public void initialize() {
        defaultAngle = Degrees.of(20);
    }

    @Override
    public void execute() {
        if(HeadingController.HEADING_KP.hasChanged()) {
            debug("Changing KP!");
            drive.HeadingController.setP(HeadingController.HEADING_KP.get());
        }

        if(HeadingController.HEADING_KD.hasChanged()) {
            debug("Changing KD!");
            drive.HeadingController.setD(HeadingController.HEADING_KD.get());
        }
        

        

        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
        var offset = Constants.ShooterConstants.speakerArmOffset.magnitude();
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        
        if (!hasTarget) {
            debug(" "+ hasTarget);
            return;
        }
        Pose3d target = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
        var distance = target.getTranslation().getDistance(new Translation3d());
        var angle = Units.radiansToDegrees(target.getRotation().getAngle());
        // if (distance == 0) {
        //     debug("Shooter:", "wtf?");
        //     shooterSubsystem.getIo().setArmPosition(defaultAngle);
        //     return;
        // }

        ShootingParameters parameter  = ShootingParametersTable.getInstance().getParameters(distance);
        debug("Shooter:",
                " distance => " + distance + " angle => " + parameter.getAngle());
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);

        // Calculated using highly-sophisticated software.
        // Do not touch unless you (really) know what you're doing!
        // double A = -317.1;
        // double B = 631.7;
        // double C = -489.2;
        // double D = 200.1;
        // double E = -40.88;
        // double F = 3.268;
        // offset =  A + B * distance + C * Math.pow(distance, 2) + D * Math.pow(distance, 3) + E * Math.pow(distance, 4) + F * Math.pow(distance, 5);
        // double A1 = 18.43145;
        // double A2 = 67.62172;
        // double x0 = 2.07751;
        // double p = 5.16297;
        // offset = A2 + (A1 - A2) / (1 + Math.pow(distance / x0, p));
        // double A1 = 69.6287;
        // double A2 = 11.4576;
        // double x0 = 1.99428;
        // double p = -4.33742;
        // offset = A2 + (A1 - A2) / (1 + Math.pow(distance / x0, p)) + 1. ;

        
        
        // debug("Shooter:", "desired angle = " + offset);
        // //debug("Shooter:", "actual angle = " + shooterSubsystem.getInputs().armPosition.in(Degrees));
        SmartDashboard.putNumber("shooter desired angle", Units.degreesToRadians(parameter.getAngle()));
        SmartDashboard.putNumber("heading angle", swerveSubsystem.getPigeon2().getRotation2d().getDegrees());
        SmartDashboard.putNumber("tag angle", swerveSubsystem.getPigeon2().getRotation2d().getDegrees()
         + LimelightHelpers.getTX("limelight"));


        if (0 > offset || offset > 180) {
            debug("Shooter:", "wtf?");
            shooterSubsystem.getIo().setArmPosition(defaultAngle);
            return;
        }

        if (Math.abs(
                offset -
                        shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 0.5) {
            defaultAngle = Degrees.of(offset);
            shooterSubsystem
                    .getIo()
                    .setArmPosition(
                            Radians.of(
                                    Units.degreesToRadians(parameter.getAngle()))
                    );
        }

        swerveSubsystem.applyRequest(() ->
                drive
                        .withVelocityX(Utils.sign(-driverController.getLeftY())
                                * xLimiter.calculate(Math.abs(driverController.getLeftY()))
                                * maxSpeed.magnitude())
                        .withVelocityY(
                                Utils.sign(-driverController.getLeftX()) * maxSpeed.magnitude()
                                        * yLimiter.calculate(Math.abs(driverController.getLeftX()))
                        )
                        .withCurrentTx(LimelightHelpers.getTX("limelight") * 1.6)
        ).execute();

        //debug(" " + Constants.HeadingController.HEADING_KP.get());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }
}
