package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class SetConveyorGate extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor conveyorSubsystem;
  private boolean commandDone = false;
  private boolean gateState;

  public SetConveyorGate(Conveyor conveyor, boolean state) {
    this.conveyorSubsystem = conveyor;
    this.gateState = state;

    addRequirements(conveyor);
  }

  @Override
  public void execute() {
    conveyorSubsystem.setGate(gateState);
    commandDone = true;
  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
