package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final MotorController intakeMotor;
  private final Solenoid deploy;

  public Intake(MotorController intakeMotor, Solenoid deploy) {

    this.intakeMotor = intakeMotor;
    this.deploy = deploy;

  }

  public void runIntake(double power) {
    intakeMotor.set(power);
  }

  public void setDeploy(boolean state) {
    deploy.set(state);
  }

  public boolean getDeploy() {
    return deploy.get();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
