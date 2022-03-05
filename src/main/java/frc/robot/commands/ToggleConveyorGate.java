package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ToggleConveyorGate extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor conveyorSubsystem;
  private boolean commandDone = false;

  public ToggleConveyorGate(Conveyor conveyor) {
    this.conveyorSubsystem = conveyor;

    addRequirements(conveyor);
  }

  @Override
  public void execute() {

    conveyorSubsystem.setGate(!conveyorSubsystem.getGate());
    commandDone = true;

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
