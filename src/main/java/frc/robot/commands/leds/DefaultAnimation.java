// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefaultAnimation extends SequentialCommandGroup {
  /** Creates a new SetIndividual. */
  public DefaultAnimation() {
    addRequirements(RobotContainer.ledSubsystem);

    for (var i = 0; i < 60; i++) {
      final int currentLED = i;
      if (i % 2 == 0) {
        addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(currentLED, Color.kRed)));
      } else {
        addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setIndividual(currentLED, Color.kWhite)));
      }
      addCommands(new WaitCommand(0.05));
    }
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
