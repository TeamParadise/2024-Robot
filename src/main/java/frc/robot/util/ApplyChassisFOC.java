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
public class ApplyChassisFOC implements SwerveRequest {
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
     * The chassis speeds to apply to the drivetrain.
     */
    public ChassisSpeeds Speeds = new ChassisSpeeds();
    /**
     * The center of rotation to rotate around.
     */
    public Translation2d CenterOfRotation = new Translation2d(0, 0);
    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.ClosedLoopOutputType DriveRequestType = SwerveModule.ClosedLoopOutputType.TorqueCurrentFOC;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
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
     * Sets the chassis speeds to apply to the drivetrain.
     *
     * @param speeds Chassis speeds to apply to the drivetrain
     * @return this request
     */
    public ApplyChassisFOC withSpeeds(ChassisSpeeds speeds) {
        this.Speeds = speeds;
        return this;
    }
    /**
     * Sets the center of rotation to rotate around.
     *
     * @param centerOfRotation Center of rotation to rotate around
     * @return this request
     */
    public ApplyChassisFOC withCenterOfRotation(Translation2d centerOfRotation) {
        this.CenterOfRotation = centerOfRotation;
        return this;
    }

    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public ApplyChassisFOC withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }
}