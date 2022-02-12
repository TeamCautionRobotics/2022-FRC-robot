package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private final MotorController conveyorMotor;
  private final Solenoid gatePiston;


  public Conveyor(MotorController conveyorMotor, Solenoid gatePiston) {

    this.conveyorMotor = conveyorMotor;
    this.gatePiston = gatePiston;

  }

  public void setConveyorMotor(double power) {
    conveyorMotor.set(power);
  }

  public void setGatePiston(boolean state) {
    gatePiston.set(state);
  }

  public boolean getGatePiston() {
    return gatePiston.get();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
