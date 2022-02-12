package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeMotor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_subsystem;

  private double power;

  public SetIntakeMotor(Intake subsystem, double power) {
    m_subsystem = subsystem;

    this.power = power;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {

    m_subsystem.runIntake(power);

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
