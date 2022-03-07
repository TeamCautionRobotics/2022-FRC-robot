package frc.robot.commands;

import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_CalibrateArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;

  private int climbStep = 0;


  /**
   * creates a new Climb_LiftAndLatch commnad.
   * pulls down and hooks onto the first bar, lifts and latches using the static hooks.
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_CalibrateArm(ClimbAngle angleSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // start on step 0 for calibration
    climbStep = 0;

    commandDone = false;

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 0:

        commandDone = false;
        if (angleSubsystem.getLeftArmAtZeroSwitch() && angleSubsystem.getRightArmAtZeroSwitch()) {
          angleSubsystem.setPower(0.0);
          angleSubsystem.setEncoderPosition(0.0);
          climbStep = 1;
        } else {
          angleSubsystem.enablePID(false);
          angleSubsystem.setPower(-0.2);
        }
        break;

      case 1:  // Calibrate the lift encoders

        if (liftSubsystem.getLeftArmFullyDownSwitch() && liftSubsystem.getRightArmFullyDownSwitch()) {
          liftSubsystem.setPower(0.0);  // stop the motors
          liftSubsystem.setEncoderPosition(0.0);  // zero the encoders
          climbStep = 3;  // go to next step
        } else {
          liftSubsystem.enablePID(false);  // disable the PID
          liftSubsystem.setPower(-0.1);  // pull down
        }  
        break;

      case 3:

        commandDone = true;  // we're done
        break;

    }

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
