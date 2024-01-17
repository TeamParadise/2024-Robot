// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.counter.ExternalDirectionCounter;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.ExtendElevator;
import frc.robot.commands.elevator.RetractElevator;
import frc.robot.commands.flywheel.PIDFlywheel;
import frc.robot.commands.flywheel.StopFlywheel;
import frc.robot.commands.primer.FeedNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmp extends SequentialCommandGroup {
  /** Creates a new GroundPickup. */
  public ScoreAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PIDFlywheel(Constants.SpeedConstants.ampSpeed).withTimeout(1).alongWith(new ExtendElevator()),
      new FeedNote().withTimeout(1),
      new RetractElevator(),
      new StopFlywheel()
    );
  }
}
