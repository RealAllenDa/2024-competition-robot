package net.ironpulse.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import net.ironpulse.Constants;

@SuppressWarnings("PMD.DataflowAnomalyAnalysis")
public class FieldCentricHeading implements SwerveRequest {
    /**
     * The velocity in the Y direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     */
    public double VelocityX = 0;

    /**
     * Current tx returned by Limelight
     */
    public double VelocityY = 0;

    public Rotation2d angleCurrent = new Rotation2d();

    public Rotation2d angleTarget = new Rotation2d();

    /**
     * The allowable deadband of the request.
     */
    public double Deadband = 0;

    /**
     * The rotational deadband of the request.
     */
    public double RotationalDeadband = 0;

    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;

    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;
    public PhoenixPIDController HeadingController = new PhoenixPIDController(140, 1, 1);

    @Override
    public StatusCode apply(
            SwerveControlRequestParameters parameters,
            SwerveModule... modulesToApply) {

        var ks = Constants.HeadingController.HEADING_KS.get();
        var toApplyX = VelocityX;
        var toApplyY = VelocityY;
        var toApplyOmega = HeadingController.calculate(
                angleCurrent.getDegrees(), angleTarget.getDegrees(),
                parameters.timestamp
        );
        if(Math.abs(angleCurrent.getDegrees() - angleTarget.getDegrees()) >= 0.3){
           toApplyOmega += Math.signum(toApplyOmega) * ks;
        }
        if (Math.hypot(toApplyX, toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }

        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
        }

        var speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                parameters.currentPose.getRotation()), parameters.updatePeriod);

        var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (var i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
        }

        return StatusCode.OK;
    }

    /**
     * Sets the velocity in the X direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     *
     * @param velocityX Velocity in the X direction, in m/s
     * @return this request
     */
    public FieldCentricHeading withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    /**
     * Sets the desired direction to face.
     * 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     *
     * @param velocityY Desired direction to face
     * @return this request
     */
    public FieldCentricHeading withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public FieldCentricHeading withDeadband(double deadband) {
        this.Deadband = deadband;
        return this;
    }

    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public FieldCentricHeading withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive
     *                         motor
     * @return this request
     */
    public FieldCentricHeading withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }

    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer
     *                         motor
     * @return this request
     */
    public FieldCentricHeading withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }

    /**
     * The angular rate to rotate at, in radians per second.
     * Angular rate is defined as counterclockwise positive,
     * so this determines how fast to turn counterclockwise.
     *
     * @param currentTx Angular rate to rotate at, in radians per second
     * @return this request
     */
    public FieldCentricHeading withCurrentAngle(Rotation2d CurrentAngle) {
        this.angleCurrent = CurrentAngle;
        return this;
    }

    public FieldCentricHeading withTargetAngle(Rotation2d targetAngle) {
        this.angleTarget = targetAngle;
        return this;
    }
}
