package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class SetConveyorMotor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Conveyor conveyor;
  private double conveyorPower = 0.0;

  public SetConveyorMotor(Conveyor conveyor, double power) {
    this.conveyor = conveyor;
    this.conveyorPower = power;

    addRequirements(conveyor);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    conveyor.setConveyorMotor(conveyorPower);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
