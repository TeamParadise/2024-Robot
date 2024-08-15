// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterOnly;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSpeakerShoot extends Command {
  private final ShooterSubsystem shooter = RobotContainer.m_shooterSubsystem;
  private final PrimerSubsystem primer = RobotContainer.m_primerSubsystem;

  // Create items to track the amount of time the command has been running for
  private double timeElapsed = 0.0;
  private double previousTimestamp = 0.0;

  private Debouncer noteDebouncer = new Debouncer(0.75, DebounceType.kBoth);

  private final SparkPIDController leftPIDController = shooter.leftShooter.getPIDController();
  private final SparkPIDController rightPIDController = shooter.rightShooter.getPIDController();

  private boolean noteShooting = false;
  /** Creates a new SpeakerShoot. */
  public AutoSpeakerShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, primer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteShooting = false;
    noteDebouncer = new Debouncer(0.75, DebounceType.kBoth);
    // Reset time elapsed values
    timeElapsed = 0.0;
    previousTimestamp = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentShooterVelocity = shooter.getAverageVelocity();

    // Set speed of flywheels.
    leftPIDController.setReference(5000, CANSparkBase.ControlType.kVelocity);
    rightPIDController.setReference(-5000, CANSparkBase.ControlType.kVelocity);
    
    if (currentShooterVelocity > 2200 && RobotContainer.checkIntersection() && RobotContainer.getRobotPointedToSpeaker()) {
      noteShooting = true;
      primer.setSpeed(0.75);
    } else if (timeElapsed < 150000) {
      primer.setSpeed(-0.15);
    } else {
      primer.setSpeed(0);
    };

    // Calculate the time that has elapsed since the command has started
    timeElapsed += RobotController.getFPGATime() - previousTimestamp;
    previousTimestamp = RobotController.getFPGATime();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    primer.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Add end condition eventually
    return noteShooting ? noteDebouncer.calculate(!RobotContainer.m_primerSubsystem.getPrimerBeamBreaker()) : false;
  }
}
