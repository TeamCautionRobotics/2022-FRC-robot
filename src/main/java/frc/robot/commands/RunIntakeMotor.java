package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeMotor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intakeSubsystem;

  public RunIntakeMotor(Intake subsystem) {
    intakeSubsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {

    intakeSubsystem.runMotor(0.69);

  }

  @Override
  public void end(boolean interrupted) {

    intakeSubsystem.runMotor(0.0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
