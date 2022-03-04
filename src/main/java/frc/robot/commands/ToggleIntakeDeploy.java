package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIntakeDeploy extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_subsystem;
  private boolean commandFinished = false;

  public ToggleIntakeDeploy(Intake subsystem) {
    m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.setDeploy(!m_subsystem.getDeploy());
    commandFinished = true;
  }

  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
