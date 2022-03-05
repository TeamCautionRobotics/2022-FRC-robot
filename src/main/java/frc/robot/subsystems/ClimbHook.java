package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbHook extends SubsystemBase {

  private final Solenoid hookPiston;

  public ClimbHook(Solenoid hookPiston) {
    this.hookPiston = hookPiston;
  }

  /**
   * Sets the static hook's state. it is inverted on the bot!
   * @param state false: in, true: out
   */
  public void setHook(boolean state) {
    hookPiston.set(!state);
  }
  
  /**
   * @return the state of the hook solenoid
   */
  public boolean getHook() {
    return hookPiston.get();
  }

}