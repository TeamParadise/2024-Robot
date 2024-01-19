// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.primeNote;
import frc.robot.commands.leds.blinkOrange;
import frc.robot.commands.leds.solidOrange;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.primer;


public class RobotContainer {

  
  public RobotContainer() {
    configureBindings();
  }

  CommandXboxController driverXbox = new CommandXboxController(0);
  public final static  LEDSubsystem ledSubsystem = new LEDSubsystem(); 
  public final static  primer primerSubsystem = new primer(); 

  private void configureBindings() {
    driverXbox.a().whileTrue(new primeNote());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
