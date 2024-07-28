// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.ShooterOnly;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAuto extends Command {
  // Subsystems used in this command
  private final ShooterSubsystem shooter = RobotContainer.m_shooterSubsystem;
  private final PrimerSubsystem primer = RobotContainer.m_primerSubsystem;

  private double primerFeedTime = 2000000;

  // Create items to track the amount of time the command has been running for
  private double timeElapsed = 0.0;
  private double previousTimestamp = 0.0;

  private double shooterRPM;

  // Used to reduce false positives or negatives by making sure something is true for long enough.
  // This is used to detect if our flywheels are up to speed
  private Debouncer shooterDebouncer = new Debouncer(0.1, DebounceType.kBoth);

  private final SparkPIDController leftPIDController = shooter.leftShooter.getPIDController();
  private final SparkPIDController rightPIDController = shooter.rightShooter.getPIDController();


  double setpoint, output, positionDegrees, positionRadians, velocity;
  double MaxSpeed = 6;

  /*feedforwardMax is the max volts that need to be supplied to match gravity (arm at 0 degrees)*/
  public double feedforwardMax = 0;
  public ShooterAuto(double shooterRPM) {
    addRequirements(shooter, primer);
    this.shooterRPM = shooterRPM;
  }

  @Override
  public void initialize() {

    shooterDebouncer = new Debouncer(0.1, DebounceType.kBoth);
    
    // Reset time elapsed values
    timeElapsed = 0.0;
    previousTimestamp = RobotController.getFPGATime();
  }

  @Override
  public void execute() {
    // Set flywheel RPM and flywheel velocity for this loop
    double shooterSetpoint = shooterRPM;
    double currentShooterVelocity = shooter.getAverageVelocity();
    SmartDashboard.putNumber("current shooter velocity", currentShooterVelocity);
    SmartDashboard.putNumber("shooter setpoint", shooterSetpoint);
    SmartDashboard.putBoolean("Shooter Debouncer", shooterDebouncer.calculate(currentShooterVelocity > shooterSetpoint - Math.pow(shooterSetpoint, 0.7) && currentShooterVelocity < shooterSetpoint + Math.pow(shooterSetpoint, 0.7)));


    // Set velocity of flywheels and intake depending on time elapsed here
    if (timeElapsed < primerFeedTime && !(shooterDebouncer.calculate(currentShooterVelocity > shooterSetpoint - Math.pow(shooterSetpoint, 0.75) && currentShooterVelocity < shooterSetpoint + Math.pow(shooterSetpoint, 0.75)))) {
      leftPIDController.setReference(shooterSetpoint, CANSparkBase.ControlType.kVelocity);
      rightPIDController.setReference(-shooterSetpoint, CANSparkBase.ControlType.kVelocity);
    } else {
      // Just set time elapsed to 4 seconds if it is not 4 seconds already, so there is 1 second
      // left to shoot the note
      if (timeElapsed < primerFeedTime) {
        timeElapsed = primerFeedTime;
      }

      leftPIDController.setReference(shooterSetpoint, CANSparkBase.ControlType.kVelocity);
      rightPIDController.setReference(-shooterSetpoint, CANSparkBase.ControlType.kVelocity);

      primer.setSpeed(SpeedConstants.kPrime);
    }

    // Calculate the time that has elapsed since the command has started
    timeElapsed += RobotController.getFPGATime() - previousTimestamp;
    previousTimestamp = RobotController.getFPGATime();
  }

  @Override
  public boolean isFinished() {
    return (timeElapsed >= 2100000 && !RobotContainer.primerBeamTrigger.getAsBoolean()) || timeElapsed > 2500000 ;
  }

  @Override
  public void end(boolean interrupted) {
    primer.setSpeed(0);
    shooter.setSpeed(0);
  }
}
