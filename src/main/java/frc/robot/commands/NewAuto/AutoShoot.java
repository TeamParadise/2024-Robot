// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.NewAuto;

import java.util.Optional;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends Command {
  // Subsystems used in this command
  private final ArmSubsystem arm = RobotContainer.m_ArmSubsystem;
  private final ShooterSubsystem shooter = RobotContainer.m_shooterSubsystem;
  private final PrimerSubsystem primer = RobotContainer.m_primerSubsystem;
  private final CommandSwerveDrivetrain drive = RobotContainer.drivetrain;

  private double shooterRPM;
  private double armPos;
  private double primerFeedTime = 4000000;

  // Create items to track the amount of time the command has been running for
  private double timeElapsed = 0.0;
  private double previousTimestamp = 0.0;

  // Used to reduce false positives or negatives by making sure something is true for long enough.
  // This is used to detect if our flywheels are up to speed
  private final Debouncer shooterDebouncer = new Debouncer(0.20, DebounceType.kBoth);
  private final Debouncer armDebouncer = new Debouncer(0.20, DebounceType.kBoth);

  private final SparkPIDController leftPIDController = shooter.leftShooter.getPIDController();
  private final SparkPIDController rightPIDController = shooter.leftShooter.getPIDController();


  double setpoint, output, positionDegrees, positionRadians, velocity;

  private final double 
    kp = .85, 
    ki = 0, 
    kd = 0,
    maxAccel = .5,
    maxVelo = 1;
  
  double MaxSpeed = 6;

  /*feedforwardMax is the max volts that need to be supplied to match gravity (arm at 0 degrees)*/
  public double feedforwardMax = 0;
  PIDController armController = new PIDController(kp, ki, kd);

  public AutoShoot() {
    addRequirements(arm, shooter, primer, drive);
  }

  @Override
  public void initialize() {
    armController.enableContinuousInput(0, 360);

    // Reset time elapsed values
    timeElapsed = 0.0;
    previousTimestamp = RobotController.getFPGATime();
  }

  @Override
  public void execute() {
    // Set flywheel RPM and flywheel velocity for this loop
    double shooterSetpoint = Robot.m_ArmLUTRPM.get(arm.getDistance());
    double currentShooterVelocity = shooter.getAverageVelocity();

    // arm
    double armSetpoint = MathUtil.clamp(Robot.m_ArmLUTAngle.get(arm.getDistance()), 0, 55);
    double currentArmPosition = RobotContainer.m_intakeSubsystem.getArmPosition();

    // Set velocity of flywheels and intake depending on time elapsed here
    if (timeElapsed < 4000000 && !(shooterDebouncer.calculate(currentShooterVelocity > shooterSetpoint - Math.pow(shooterSetpoint, 0.635) && currentShooterVelocity < shooterSetpoint + Math.pow(shooterSetpoint, 0.635))
        && !armDebouncer.calculate(currentArmPosition > armSetpoint - Math.pow(armSetpoint, 0.2) && currentArmPosition < armSetpoint - Math.pow(armSetpoint, 0.2)))) {
      leftPIDController.setReference(shooterSetpoint, CANSparkBase.ControlType.kVelocity);
      rightPIDController.setReference(-shooterSetpoint, CANSparkBase.ControlType.kVelocity);
      output = armController.calculate(currentArmPosition, armSetpoint) + feedforwardMax * Math.cos(Math.toRadians(Math.toRadians(positionDegrees)));
      arm.setVoltage(MathUtil.clamp(output, -9, 9));
      drive.setControl(RobotContainer.headingDrive.withTargetDirection(DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? new Rotation2d(Math.atan2(5.475 - drive.getState().Pose.getY(),  (16.541748) - drive.getState().Pose.getX())) : new Rotation2d(Math.atan2(5.475 - drive.getState().Pose.getY(),  (0) - drive.getState().Pose.getX()))));
    } else {
      // Just set time elapsed to 4 seconds if it is not 4 seconds already, so there is 1 second
      // left to shoot the note
      if (timeElapsed < 4000000) {
        timeElapsed = 4000000;
      }

      leftPIDController.setReference(shooterSetpoint, CANSparkBase.ControlType.kVelocity);
      rightPIDController.setReference(-shooterSetpoint, CANSparkBase.ControlType.kVelocity);
      output = armController.calculate(currentArmPosition, armSetpoint) + feedforwardMax * Math.cos(Math.toRadians(Math.toRadians(positionDegrees)));
      arm.setVoltage(MathUtil.clamp(output, -9, 9));

      primer.setSpeed(SpeedConstants.kPrime);
    }

    // Calculate the time that has elapsed since the command has started
    timeElapsed += RobotController.getFPGATime() - previousTimestamp;
    previousTimestamp = RobotController.getFPGATime();
  }

  @Override
  public boolean isFinished() {
    return timeElapsed >= 5000000;
  }

  @Override
  public void end(boolean interrupted) {
    primer.setSpeed(0);
    shooter.setSpeed(0);
    arm.setVoltage(0);
    drive.setControl(RobotContainer.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
}
