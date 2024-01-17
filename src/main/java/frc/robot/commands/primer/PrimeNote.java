// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class PrimeNote extends Command {
  /** Creates a new PrimeNote. */
  public PrimeNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.primerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running Primer");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.primerSubsystem.runMotors(0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Retracting Primer");
    RobotContainer.primerSubsystem.runMotors(-0.25);
    new WaitCommand(0.025);
    System.out.println("Stopping Primer");
    RobotContainer.primerSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
