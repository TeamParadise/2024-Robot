// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterOnly;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PrimerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmp extends Command {
  private final ShooterSubsystem shooter = RobotContainer.m_shooterSubsystem;
  private final PrimerSubsystem primer = RobotContainer.m_primerSubsystem;
  private final ElevatorSubsystem elevator = RobotContainer.m_ElevatorSubsystem;

  private final SparkPIDController leftPIDController = shooter.leftShooter.getPIDController();
  private final SparkPIDController rightPIDController = shooter.rightShooter.getPIDController();

  PIDController elevatorController;

  double elevatorSetpoint;

  private Debouncer noteDebouncer = new Debouncer(0.5, DebounceType.kBoth);


  /** Creates a new AmpCommand. */
  public ShootAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, primer, elevator);

    elevatorController = new PIDController(0.5, 0, .00025);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorController.reset();
    noteDebouncer = new Debouncer(0.5, DebounceType.kBoth);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set speed of flywheels.
    leftPIDController.setReference(SmartDashboard.getNumber("Amp Flywheel Speed", 0), CANSparkBase.ControlType.kVelocity);
    rightPIDController.setReference(-SmartDashboard.getNumber("Amp Flywheel Speed", 0), CANSparkBase.ControlType.kVelocity);

    // Get setpoint of elevator and set it.
    elevatorSetpoint = MathUtil.clamp(SmartDashboard.getNumber("Amp Elevator Setpoint", 0), 0, 52.5);
    elevatorController.setSetpoint(elevatorSetpoint);
    double output = elevatorController.calculate(elevator.getEncoder());
    elevator.setSpeed(MathUtil.clamp(output, -0.4, 0.4));  

    // Set primer speed
    primer.setSpeed(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    primer.setSpeed(0);
    elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteDebouncer.calculate(!RobotContainer.m_primerSubsystem.getPrimerBeamBreaker());
  }
}
