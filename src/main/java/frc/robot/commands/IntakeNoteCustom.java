// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.Arm.armPID;
import frc.robot.commands.Intake.intakeController;
import frc.robot.commands.Intake.intakePIDF;
import frc.robot.commands.Primer.PrimeNote;
import frc.robot.commands.Shooter.shooterPIDF;
import frc.robot.commands.Shooter.shooterSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteCustom extends SequentialCommandGroup {
  /** Creates a new IntakeNote. */
  public IntakeNoteCustom() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(//new armPID(35).withTimeout(1),
                new armPID(15).alongWith(new intakePIDF(4250).alongWith(new PrimeNote(0.45)).alongWith(new shooterPIDF(-2500))));
                
        
  }
}
