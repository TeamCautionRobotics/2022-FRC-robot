package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ToggleConveyorGate extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor conveyor;

  public ToggleConveyorGate(Conveyor conveyor) {
    this.conveyor = conveyor;

    addRequirements(conveyor);
  }

  @Override
  public void initialize() {

    conveyor.setGatePiston(!conveyor.getGatePiston());

  }

  @Override
  public void execute() {  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
