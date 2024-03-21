// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class intakePIDF extends Command {
  /** Creates a new intakeController. */

  private final SparkPIDController intakePIDController;
  private double setpoint;
  
  double rpm;
  public intakePIDF(double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intakeSubsystem);
    
    intakePIDController = RobotContainer.m_intakeSubsystem.intakeMotor.getPIDController();

    SmartDashboard.putNumber("intakeP", IntakeConstants.kLeftP);
    SmartDashboard.putNumber("intakeI", IntakeConstants.kLeftI);
    SmartDashboard.putNumber("intakeD", IntakeConstants.kLeftD);
    SmartDashboard.putNumber("intakeIZ", IntakeConstants.kLeftIz);
    SmartDashboard.putNumber("intakeFF", IntakeConstants.kLeftFF);
    // SmartDashboard.putNumber("intake Setpoint", 0);
    intakePIDController.setOutputRange(-1, 1);
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakePIDController.setP(SmartDashboard.getNumber("intakeP", IntakeConstants.kLeftP));
    intakePIDController.setI(SmartDashboard.getNumber("intakeI", IntakeConstants.kLeftI));
    intakePIDController.setD(SmartDashboard.getNumber("intakeD", IntakeConstants.kLeftD));
    intakePIDController.setIZone(SmartDashboard.getNumber("intakeIZ", IntakeConstants.kLeftIz));
    intakePIDController.setFF(SmartDashboard.getNumber("intakeFF", IntakeConstants.kLeftFF));
    // rpm = SmartDashboard.getNumber("intake Setpoint", 0);
    // SmartDashboard.putNumber("intake speed", RobotContainer.m_intakeSubsystem.intakeMotor.getEncoder().getVelocity());
    intakePIDController.setReference(-rpm, CANSparkBase.ControlType.kVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
