// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Breathe extends SequentialCommandGroup {
  /** Creates a new Breathe. */
  public Breathe(int hue, int saturation, int value) {
    addRequirements(RobotContainer.ledSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    for (var i = 0; i <= value; i += 2) {
      final int currentBrightness = i;
      addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setAll(hue, saturation, currentBrightness > value ? value : currentBrightness)));
    }
    for (var i = value; i >= 0; i -= 2) {
      final int currentBrightness = i;
      addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setAll(hue, saturation, currentBrightness < 0 ? 0 : currentBrightness)));
    }
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
