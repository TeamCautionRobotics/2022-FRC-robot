package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeDeploy extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_subsystem;
  private boolean commandFinished = false;

  private boolean intakeState;

  public SetIntakeDeploy(Intake subsystem, boolean state) {
    m_subsystem = subsystem;

    intakeState = state;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {

    m_subsystem.setDeploy(intakeState);
    commandFinished = true;

  }

  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
