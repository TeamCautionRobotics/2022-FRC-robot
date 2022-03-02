package frc.robot.commands;

import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbAngle;
import frc.robot.subsystems.ClimbHook;
import frc.robot.subsystems.ClimbLift;

public class Climb_Testing extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final ClimbAngle angleSubsystem;
  private final ClimbHook hookSubsystem;
  private final ClimbLift liftSubsystem;

  // angle vars
  private boolean anglePidEnabled = true;
  private double angleSetpoint = 0;
  private double angle_kP = Constants.Climb.angler.kP;
  private double angle_kI = Constants.Climb.angler.kI;
  private double angle_kD = Constants.Climb.angler.kD;

  private boolean anglePidEnabled_last = true;
  private double angleSetpoint_last = 0;
  private double angle_kP_last = Constants.Climb.angler.kP;
  private double angle_kI_last = Constants.Climb.angler.kI;
  private double angle_kD_last = Constants.Climb.angler.kD;

  // lift vars
  private double liftSetpoint = 0;
  private ControlType liftSetpointType = ControlType.kPosition;
  private double lift_kP = Constants.Climb.lift.kP;
  private double lift_kI = Constants.Climb.lift.kI;
  private double lift_kD = Constants.Climb.lift.kD;
  private double lift_kIz = Constants.Climb.lift.kIz;
  private double lift_kFF = Constants.Climb.lift.kFF;

  private double liftSetpoint_last = 0;
  private double lift_kP_last = Constants.Climb.lift.kP;
  private double lift_kI_last = Constants.Climb.lift.kI;
  private double lift_kD_last = Constants.Climb.lift.kD;
  private double lift_kIz_last = Constants.Climb.lift.kIz;
  private double lift_kFF_last = Constants.Climb.lift.kFF;

  private SendableChooser<ControlType> liftSetpointTypeChooser = new SendableChooser<>();

  // hook vars
  private boolean hookEnabled = false;
  private boolean hookEnabled_last = false;


  public Climb_Testing(ClimbAngle angleSubsystem, ClimbHook hookSubsystem, ClimbLift liftSubsystem) {
    addRequirements(angleSubsystem, hookSubsystem, liftSubsystem);

    this.angleSubsystem = angleSubsystem;
    this.hookSubsystem = hookSubsystem;
    this.liftSubsystem = liftSubsystem;

    liftSetpointTypeChooser.setDefaultOption("Lift: kPosition", ControlType.kPosition);
    liftSetpointTypeChooser.addOption("Lift: kVelocity", ControlType.kVelocity);

  }

  @Override
  public void initialize() {

    // put numbers on the smart dash
    // angle vars
    SmartDashboard.putBoolean("Angle PID Enable", anglePidEnabled);
    SmartDashboard.putNumber("Angle Setpoint", angleSetpoint);
    SmartDashboard.putNumber("Angle P", angle_kP);
    SmartDashboard.putNumber("Angle I", angle_kI);
    SmartDashboard.putNumber("Angle D", angle_kD);

    // lift vars
    SmartDashboard.putNumber("Lift Setpoint", liftSetpoint);
    SmartDashboard.putData(liftSetpointTypeChooser);
    SmartDashboard.putNumber("Lift P", liftSetpoint);
    SmartDashboard.putNumber("Lift I", lift_kP);
    SmartDashboard.putNumber("Lift D", lift_kI);
    SmartDashboard.putNumber("Lift I-Zone", lift_kD);
    SmartDashboard.putNumber("Lift Feedforward", lift_kFF);

    // hook vars
    SmartDashboard.putBoolean("Hook Deployed", hookEnabled);

  }

  @Override
  public void execute() {

    // print current angle vars
    SmartDashboard.putNumber("Angle Left Distance", angleSubsystem.getLeftAngleEncoderDistance());
    SmartDashboard.putNumber("Angle Right Distance", angleSubsystem.getRightAngleEncoderDistance());

    SmartDashboard.putBoolean("Angle Left Sw", angleSubsystem.getLeftArmAtZeroSwitch());
    SmartDashboard.putBoolean("Angle Right Sw", angleSubsystem.getRightArmAtZeroSwitch());
  
    // update angle vars
    anglePidEnabled =  SmartDashboard.getBoolean("Angle PID Enable", false);
    angleSetpoint = SmartDashboard.getNumber("Angle Setpoint", 0.0);
    angle_kP = SmartDashboard.getNumber("Angle P", 0.0);
    angle_kI = SmartDashboard.getNumber("Angle I", 0.0);
    angle_kD = SmartDashboard.getNumber("Angle D", 0.0);

    if (anglePidEnabled != anglePidEnabled_last) {
      anglePidEnabled_last = anglePidEnabled;
      angleSubsystem.enableAnglePid(anglePidEnabled);
      System.out.println("updated angle PID enable state");
    }

    if (angleSetpoint != angleSetpoint_last) {
      angleSetpoint_last = angleSetpoint;
      angleSubsystem.setAngleReference(angleSetpoint);
      System.out.println("updated angle setpoint");
    }

    if ((angle_kP != angle_kP_last) ||
        (angle_kI != angle_kI_last) ||
        (angle_kD != angle_kD_last)) {
          angle_kP_last = angle_kP;
          angle_kI_last = angle_kI;
          angle_kD_last = angle_kD;
          angleSubsystem.setPidVars(angle_kP, angle_kI, angle_kD);
          System.out.println("updated angle PID vars");
        }

    // print current lift vars
    SmartDashboard.putNumber("Lift Left Distance", liftSubsystem.getLeftLiftPosition());
    SmartDashboard.putNumber("Lift Right Distance", liftSubsystem.getRightLiftPosition());

    SmartDashboard.putNumber("Lift Left Velocity", liftSubsystem.getLeftLiftVelocity());
    SmartDashboard.putNumber("Lift Right Velocity", liftSubsystem.getRightLiftVelocity());

    SmartDashboard.putNumber("Lift Left Current Draw", liftSubsystem.getMotorCurrent(0));
    SmartDashboard.putNumber("Lift Right Current Draw", liftSubsystem.getMotorCurrent(1));

    SmartDashboard.putBoolean("Lift Left Sw", liftSubsystem.getLeftArmFullyDownSwitch());
    SmartDashboard.putBoolean("Lift Right Sw", liftSubsystem.getRightArmFullyDownSwitch());
    
    // update lift vars
    liftSetpoint = SmartDashboard.getNumber("Lift Setpoint Setpoint", 0.0);
    lift_kP = SmartDashboard.getNumber("Lift P", 0.0);
    lift_kI = SmartDashboard.getNumber("Lift I", 0.0);
    lift_kD = SmartDashboard.getNumber("Lift D", 0.0);
    lift_kIz = SmartDashboard.getNumber("Lift I-Zone", 0.0);
    lift_kFF = SmartDashboard.getNumber("Lift Feedforward", 0.0);

    if (liftSetpoint != liftSetpoint_last) {
      liftSetpoint_last = liftSetpoint;
      liftSubsystem.setReference(liftSetpoint, liftSetpointType);
      System.out.println("updated lift setpoint");
    }

    if (liftSetpointTypeChooser.getSelected() != liftSetpointType) {
      liftSetpointType = liftSetpointTypeChooser.getSelected();
      liftSubsystem.setReference(liftSetpoint, liftSetpointType);
      System.out.println("updated lift setpoint type");
    }

    if ((lift_kP != lift_kP_last) ||
        (lift_kI != lift_kI_last) ||
        (lift_kD != lift_kD_last) ||
        (lift_kIz != lift_kIz_last) ||
        (lift_kFF != lift_kFF_last)) {

          lift_kP_last = lift_kP;
          lift_kI_last = lift_kI;
          lift_kD_last = lift_kD;
          lift_kIz_last = lift_kIz;
          lift_kFF_last = lift_kFF;

          liftSubsystem.setPidVars(lift_kP, lift_kI, lift_kD, lift_kIz, lift_kFF);
          System.out.println("updated lift pid vars");
        }

    // update hook vars
    hookEnabled = SmartDashboard.getBoolean("Hook Deployed", false);

    if (hookEnabled != hookEnabled_last) {
      hookEnabled_last = hookEnabled;
      hookSubsystem.setHook(hookEnabled);
      System.out.println("updated hook state");
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
