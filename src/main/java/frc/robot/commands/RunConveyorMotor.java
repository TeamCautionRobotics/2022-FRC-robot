package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class RunConveyorMotor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor conveyor;

  public RunConveyorMotor(Conveyor conveyor) {
    this.conveyor = conveyor;

    addRequirements(conveyor);
  }

  @Override
  public void execute() {
    conveyor.setConveyorMotor(1.0);
  }

  @Override
  public void end(boolean interrupted) {
    conveyor.setConveyorMotor(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
