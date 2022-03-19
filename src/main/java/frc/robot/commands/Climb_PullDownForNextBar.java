package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

public class Climb_PullDownForNextBar extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem; 

  private boolean commandDone = false;
  private boolean hooksSet = false;
  private int climbStep = 0;
  private Timer currentLimitTimeout = new Timer();
  private Timer timer0 = new Timer();


  /**
   * @param angleSubsystem
   * @param hookSubsystem
   * @param liftSubsystem
   */
  public Climb_PullDownForNextBar(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

  }

  @Override
  public void initialize() {

    // wpilib bug workaround
    commandDone = false;
    hooksSet = false;

    angleSubsystem.setNeutralMode(NeutralMode.Coast);  // coast the arm motors
    liftSubsystem.setIdleMode(IdleMode.kBrake);  // brake the lift motors

    climbStep = 9;  // start on step 10

  }

  @Override
  public void execute() {

    switch(climbStep) {

      case 0:  // SAFE MODE

        System.out.println("ERROR: Winch exceeded current limit! Activating safe mode...");

        hookSubsystem.set(true);  // extend the hooks

        // brake all motors
        angleSubsystem.setNeutralMode(NeutralMode.Brake);
        liftSubsystem.setIdleMode(IdleMode.kBrake);

        // cut all power to angle
        angleSubsystem.enablePID(false);
        angleSubsystem.setPower(0);
        angleSubsystem.stop();

        // cut all power to lift
        liftSubsystem.enablePID(false);
        liftSubsystem.setPower(0);
        liftSubsystem.stop();

        commandDone = true;  // we're done
        climbStep = 1;  // do nothing for remainder of command
        break;

      case 1:
        ;  // do nothing
        break;

      case 9:  // get us off the bar

        // if we're at the setpoint
        if (liftSubsystem.getLeftEncoderDistance() < 23.2 &&
            liftSubsystem.getRightEncoderDistance() < 23.2) {

          hooksSet = false;
          climbStep = 10;
        
        } else {  // if we're not there yet

          // pull down
          liftSubsystem.enablePID(true);
          liftSubsystem.setPosition(22);

          // kill pwr to arms
          angleSubsystem.enablePID(false);
          angleSubsystem.setPower(0);

          // retract the hooks if they haven't
          // (repeatedly setting solenoids causes issues)
          if (!hooksSet) {
            hookSubsystem.set(false);
            hooksSet = true;
          }

          // reset and start the timer
          timer0.reset();
          timer0.start();

        }
        break;

      case 10:  // come down until we're clear of conveyor

        // if we're at the setpoint
        if (liftSubsystem.getLeftEncoderDistance() > 23.5 &&
            liftSubsystem.getRightEncoderDistance() > 23.5) {

          climbStep = 11;

        } else {

          liftSubsystem.enablePID(false);
          liftSubsystem.setPower(0);

        }
        break;

      case 11:  // FULL PULL BROTHERRRR

        // if we're at the setpoint
        if (liftSubsystem.getLeftEncoderDistance() < 0.2 &&
            liftSubsystem.getRightEncoderDistance() < 0.2) {

          climbStep = 12;

        } else {  // if we're not there yet

          // pull down
          liftSubsystem.enablePID(true);
          liftSubsystem.setPosition(0.0);  // go for a slight stretch on the cables

        }
        break;

      case 12:  // extend the hooks and begin the timer
        
        hookSubsystem.set(true);
        timer0.stop();
        timer0.reset();
        timer0.start();
        climbStep = 13;
        break;

      case 13:  // release hold

        if (timer0.get() > 0.5) {  // half-second timer before power release

          // kill power to winch
          liftSubsystem.enablePID(false);
          liftSubsystem.setPower(0);
          timer0.stop();
          climbStep = 14;

        }
        break;

      case 14:  // finish
        
        commandDone = true;
        climbStep = 1;  // make loop do nothing
        break;
        
    }

    // safe mode trigger
    // if we're exceeding current limit:
    if(liftSubsystem.getLeftMotorCurrent() > Constants.Climb.lift.maxCurrentThreshold || 
      liftSubsystem.getRightMotorCurrent() > Constants.Climb.lift.maxCurrentThreshold) {

        if (currentLimitTimeout.get() > Constants.Climb.lift.maxCurrentTimeout) {  // if the timeout has expired while the current is exceeded
          currentLimitTimeout.stop(); // stop the timeout timer
          climbStep = 0; // go to safe mode!
        }

    } else {  // if the current has not exceeded threshold
      currentLimitTimeout.reset();  // full reset the timer
      currentLimitTimeout.start();
    }

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return commandDone;
    // return false;  // force never-ending for testing
  }
}
