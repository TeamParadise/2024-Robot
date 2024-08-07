// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Sketchy solution, better one will be implemented when custom swerve code is finally a thing. Only supports open-loop drive motor control. */
public class FOCSwitch implements SwerveRequest {
    private final ClosedLoopOutputType m_steerClosedLoopOutput = TunerConstants.steerClosedLoopOutput;
    private final ClosedLoopOutputType m_driveClosedLoopOutput = TunerConstants.driveClosedLoopOutput;
    private final double m_driveRotationsPerMeter = (TunerConstants.kDriveGearRatio) / (2 * Math.PI * Units.inchesToMeters(TunerConstants.kWheelRadiusInches));
    private final double m_couplingRatioDriveRotorToCANcoder = TunerConstants.kCoupleRatio;
    private final double m_speedAt12VoltsMps = TunerConstants.kSpeedAt12VoltsMps;

    private final MotionMagicVoltage m_angleVoltageSetter = new MotionMagicVoltage(0).withUpdateFreqHz(0);
    private final MotionMagicTorqueCurrentFOC m_angleTorqueSetter = new MotionMagicTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final MotionMagicExpoVoltage m_angleVoltageExpoSetter = new MotionMagicExpoVoltage(0).withUpdateFreqHz(0);
    private final MotionMagicExpoTorqueCurrentFOC m_angleTorqueExpoSetter = new MotionMagicExpoTorqueCurrentFOC(0).withUpdateFreqHz(0);

    private final VoltageOut m_normal = new VoltageOut(0).withUpdateFreqHz(0);
    private final VoltageOut m_foc = new VoltageOut(0).withUpdateFreqHz(0).withEnableFOC(true);

    /**
     * The velocity in the X direction, in m/s.
     * X is defined as forward according to WPILib convention,
     * so this determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction, in m/s.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The angular rate to rotate at, in radians per second.
     * Angular rate is defined as counterclockwise positive,
     * so this determines how fast to turn counterclockwise.
     */
    public double RotationalRate = 0;
    /**
     * The allowable deadband of the request.
     */
    public double Deadband = 0;
    /**
     * The rotational deadband of the request.
     */
    public double RotationalDeadband = 0;
    /**
     * The speed at which to switch from the FOC drive mode to normal drive.
     */
    public double SwitchSpeed = 0;
    /**
     * The center of rotation the robot should rotate around.
     * This is (0,0) by default, which will rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /**
     * The slow control request to use for the drive motors. Velocity is FOC, 
     */
    public SwerveModule.ClosedLoopOutputType DriveRequestType = SwerveModule.ClosedLoopOutputType.TorqueCurrentFOC;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    /**
     * The perspective to use when determining which direction is forward.
     */
    public ForwardReference ForwardReference = SwerveRequest.ForwardReference.OperatorPerspective;

    /**
     * The last applied state in case we don't have anything to drive.
     */
    protected SwerveModuleState[] m_lastAppliedState = null;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        double toApplyX = VelocityX;
        double toApplyY = VelocityY;
        ChassisSpeeds currentChassisSpeed = parameters.currentChassisSpeed;
        if (ForwardReference == SwerveRequest.ForwardReference.OperatorPerspective) {
            /* If we're operator perspective, modify the X/Y translation by the angle */
            Translation2d tmp = new Translation2d(toApplyX, toApplyY);
            tmp = tmp.rotateBy(parameters.operatorForwardDirection);
            toApplyX = tmp.getX();
            toApplyY = tmp.getY();
        }
        double toApplyOmega = RotationalRate;
        if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            toApplyX = 0;
            toApplyY = 0;
        }
        if (Math.abs(toApplyOmega) < RotationalDeadband) {
            toApplyOmega = 0;
        }

        ChassisSpeeds speeds = ChassisSpeeds
                .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                        parameters.currentPose.getRotation()), parameters.updatePeriod);

        var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

        // Check to see if our overall chassis velocity is less than our switch speed, and set the drive mode accordingly.
        if (Math.sqrt(currentChassisSpeed.vxMetersPerSecond * currentChassisSpeed.vxMetersPerSecond + currentChassisSpeed.vyMetersPerSecond * currentChassisSpeed.vyMetersPerSecond) < SwitchSpeed) {
            DriveRequestType = SwerveModule.ClosedLoopOutputType.TorqueCurrentFOC;
        } else {
            DriveRequestType = SwerveModule.ClosedLoopOutputType.Voltage;
        }

        for (int i = 0; i < modulesToApply.length; ++i) {
            var optimized = SwerveModuleState.optimize(states[i], modulesToApply[i].getCachedPosition().angle);
            var m_steerMotor = modulesToApply[i].getSteerMotor();
            var m_driveMotor = modulesToApply[i].getDriveMotor();

            double angleToSetDeg = optimized.angle.getRotations();
            switch (SteerRequestType) {
                case MotionMagic:
                    switch (m_steerClosedLoopOutput) {
                        case Voltage:
                            m_steerMotor.setControl(m_angleVoltageSetter.withPosition(angleToSetDeg));
                            break;
    
                        case TorqueCurrentFOC:
                            m_steerMotor.setControl(m_angleTorqueSetter.withPosition(angleToSetDeg));
                            break;
                    }
                    break;
    
                case MotionMagicExpo:
                    switch (m_steerClosedLoopOutput) {
                        case Voltage:
                            m_steerMotor.setControl(m_angleVoltageExpoSetter.withPosition(angleToSetDeg));
                            break;
    
                        case TorqueCurrentFOC:
                            m_steerMotor.setControl(m_angleTorqueExpoSetter.withPosition(angleToSetDeg));
                            break;
                    }
                    break;
            }

            double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;

            /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
            /* To reduce the "skew" that occurs when changing direction */
            double steerMotorError = angleToSetDeg - m_steerMotor.getPosition().getValue();
            /* If error is close to 0 rotations, we're already there, so apply full power */
            /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
            double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
            /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
            if (cosineScalar < 0.0) {
                cosineScalar = 0.0;
            }
            velocityToSet *= cosineScalar;

            /* Back out the expected shimmy the drive motor will see */
            /* Find the angular rate to determine what to back out */
            double azimuthTurnRps = m_steerMotor.getVelocity().getValue();
            /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
            double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;
            velocityToSet += driveRateBackOut;
            velocityToSet /= m_driveRotationsPerMeter;

            switch (DriveRequestType) {
                case TorqueCurrentFOC:
                    m_driveMotor.setControl(m_foc.withOutput(velocityToSet / m_speedAt12VoltsMps * 12.0));
                    break;

                case Voltage:
                    m_driveMotor.setControl(m_normal.withOutput(velocityToSet / m_speedAt12VoltsMps * 12.0));
                    break;
            }
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
    public FOCSwitch withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    /**
     * Sets the velocity in the Y direction, in m/s.
     * Y is defined as to the left according to WPILib convention,
     * so this determines how fast to travel to the left.
     *
     * @param velocityY Velocity in the Y direction, in m/s
     * @return this request
     */
    public FOCSwitch withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
        return this;
    }

    /**
     * The angular rate to rotate at, in radians per second.
     * Angular rate is defined as counterclockwise positive,
     * so this determines how fast to turn counterclockwise.
     *
     * @param rotationalRate Angular rate to rotate at, in radians per second
     * @return this request
     */
    public FOCSwitch withRotationalRate(double rotationalRate) {
        this.RotationalRate = rotationalRate;
        return this;
    }

    /**
     * Sets the allowable deadband of the request.
     *
     * @param deadband Allowable deadband of the request
     * @return this request
     */
    public FOCSwitch withDeadband(double deadband) {
        this.Deadband = deadband;
        return this;
    }

    /**
     * Sets the rotational deadband of the request.
     *
     * @param rotationalDeadband Rotational deadband of the request
     * @return this request
     */
    public FOCSwitch withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    /**
     * Sets the center of rotation of the request
     *
     * @param centerOfRotation The center of rotation the robot should rotate
     *                         around.
     * @return this request
     */
    public FOCSwitch withCenterOfRotation(Translation2d centerOfRotation) {
        this.CenterOfRotation = centerOfRotation;
        return this;
    }

    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer
     *                         motor
     * @return this request
     */
    public FOCSwitch withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }

    public FOCSwitch withSwitchSpeed(double switchSpeed) {
        this.SwitchSpeed = switchSpeed;
        return this;
    }
}
