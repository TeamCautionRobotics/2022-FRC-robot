package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ToggleConveyorGate extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor conveyor;
  private boolean commandDone = false;

  public ToggleConveyorGate(Conveyor conveyor) {
    this.conveyor = conveyor;

    addRequirements(conveyor);
  }

  @Override
  public void execute() {

    conveyor.setGatePiston(!conveyor.getGatePiston());
    commandDone = true;

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
