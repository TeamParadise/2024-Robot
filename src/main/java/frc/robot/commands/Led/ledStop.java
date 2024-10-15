package frc.robot.commands.Led;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem;

public class ledStop extends Command{
    public final static LedSubsystem m_ledSubsystem = LedSubsystem.getInstance();
    /** Creates a new intakeController. */
  boolean sop;
  public ledStop(boolean sop) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intakeSubsystem);
    this.sop = sop;
    SmartDashboard.putBoolean("Should Stop?", sop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ledSubsystem.stop = sop;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
