package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climb_LiftArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
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
  public Climb_LiftArm(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // retract the static hooks
    hookSubsystem.setHook(false);

    // start on step 0 for calibration
    climbStep = 0;

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 0:  // Calibrate the lift encoders

        // if either arm isn't pressing the switch
        if (!liftSubsystem.getLeftArmFullyDownSwitch() || !liftSubsystem.getRightArmFullyDownSwitch()) {
          liftSubsystem.enablePID(false);  // disable the PID
          liftSubsystem.setPower(-0.1);  // pull down
        } else {  // if both arms are pressing the switch
          liftSubsystem.setPower(0.0);  // stop the motors
          liftSubsystem.setEncoderPosition(0.0);  // zero the encoders
          climbStep = climbStep + 1;  // go to next step
        }

      case 1:  // calibrate the angle encoders
        
        // if either arm isn't pressing the switch
        if (!angleSubsystem.getLeftArmAtZeroSwitch() || !angleSubsystem.getRightArmAtZeroSwitch()) {
          angleSubsystem.enableAnglePid(false);  // disable the angle pid
          angleSubsystem.setPower(-0.2);  // rotate back
        } else {  // if both arms are pressing the switch
          angleSubsystem.setPower(0.0);  // stop the motors
          angleSubsystem.setEncoderPosition(0.0);  // zero the encoder
          angleSubsystem.setPosition(0.0);  // set the PID ref to zero
          climbStep = climbStep + 1; // go to next step
        }

      case 2:

        liftSubsystem.enablePID(true);  // re-enable the lift pid
        angleSubsystem.enableAnglePid(true);  // re-enable the angle pid
        liftSubsystem.setPosition(20.0);  // winch out 15 inches
        angleSubsystem.setPosition(60);  // angle arm 60 degrees
        climbStep = climbStep + 1;  // go to next step
      
      case 3:

        commandDone = true;  // we're done

    }

  }

  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
