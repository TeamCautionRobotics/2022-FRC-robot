package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

  private final CANSparkMax lifter0;
  private final CANSparkMax lifter1;
  private final SparkMaxPIDController lifter0PID;
  private final SparkMaxPIDController lifter1PID;

  private int lifterSetpoint;
  private double kP = Constants.Climb.lift.kP;
  private double kI = Constants.Climb.lift.kI;
  
  private final MotorController angulator0;
  private final MotorControllerGroup angulators;
  
  private final Solenoid hookDeploy;

  public Climb(CANSparkMax lifter0, CANSparkMax lifter1, MotorController angulator0, Solenoid hookDeploy) {

    this.lifter0 = lifter0;
    this.lifter1 = lifter1;
    lifter0PID = lifter0.getPIDController();
    lifter1PID = lifter1.getPIDController();
    
    this.angulator0 = angulator0;
    angulators = new MotorControllerGroup(angulator0);

    this.hookDeploy = hookDeploy;

  }

  /**
   * Sets the static hook's state
   * @param state false: in, true: out
   */
  public void setHook(boolean state) {
    hookDeploy.set(state);
  }





  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
