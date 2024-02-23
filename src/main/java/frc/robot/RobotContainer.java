// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.leds.AllianceAnimation;
import frc.robot.commands.leds.Blink;
import frc.robot.commands.leds.Breathe;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
  public final static LEDSubsystem ledSubsystem = new LEDSubsystem();
  public final static CommandXboxController controller = new CommandXboxController(0);

  // public static allianceColor;

  public RobotContainer() {
    configureBindings();

    
  }

  private void configureBindings() {
    controller.a().whileTrue(new RepeatCommand(new Blink()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
