// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.leds.Blink;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
  public final static LEDSubsystem ledSubsystem = new LEDSubsystem();

  // public static allianceColor;

  public RobotContainer() {
    configureBindings();

    ledSubsystem.setDefaultCommand(new Blink(Color.kOrangeRed));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
