// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.leds.blinkOrange;
import frc.robot.commands.leds.blinkOrangeFast;
import frc.robot.commands.leds.solidOrange;
import frc.robot.commands.leds.solidPurple;
import frc.robot.commands.leds.solidRed;

public class primeNote extends Command {
  /** Creates a new primeNote. */
  public primeNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.primerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!RobotContainer.primerSubsystem.getIRSensor()){
      new InstantCommand(() -> {new blinkOrange();});
      System.out.println("Note Not Found");
    }
    else{
      new InstantCommand(() -> {new blinkOrangeFast();});
      System.out.println("Note Found");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (RobotContainer.primerSubsystem.getIRSensor()){
      new solidOrange();
    }
    else {
      new solidRed();
    }

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
