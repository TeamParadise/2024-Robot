// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class blinkRed extends SequentialCommandGroup {
  /** Creates a new blinkRed. */
  public blinkRed() {
    addRequirements(RobotContainer.ledSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> {RobotContainer.ledSubsystem.setAll(Color.kRed);}).withTimeout(0.25),
      new RunCommand(() -> {RobotContainer.ledSubsystem.setAll(Color.kBlack);}).withTimeout(0.25)
    );
  }
}