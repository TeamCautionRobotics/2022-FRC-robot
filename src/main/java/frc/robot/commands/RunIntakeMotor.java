package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeMotor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_subsystem;

  public RunIntakeMotor(Intake subsystem) {
    m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {

    m_subsystem.runIntake(1.0);

  }

  @Override
  public void end(boolean interrupted) {

    m_subsystem.runIntake(0.0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
