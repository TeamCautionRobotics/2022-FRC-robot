package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIntakeDeploy extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intakeSubsystem;
  private boolean commandFinished = false;

  public ToggleIntakeDeploy(Intake subsystem) {
    intakeSubsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.setDeploy(!intakeSubsystem.getDeploy());
    commandFinished = true;
  }

  @Override
  public boolean isFinished() {
    return commandFinished;
  }
}
