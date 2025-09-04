// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.BoxConstants;


public class BoxSubsystem extends SubsystemBase {
  // Declare and intialize motor variables to a new instance of CANSparkMax
  private final SparkMax topShooterMotor = new SparkMax(BoxConstants.kTopShooterMotorID, MotorType.kBrushless);
  private final RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
  private final SparkMax bottomShooterMotor = new SparkMax(BoxConstants.kBottomShooterMotorID, MotorType.kBrushless);
  private final RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();
  private final SparkMax intakeMotor = new SparkMax(BoxConstants.kIntakeMotorID, MotorType.kBrushless);
  // PID controllers
  private SparkClosedLoopController topShooterPIDController = topShooterMotor.getClosedLoopController();
  private SparkClosedLoopController bottomShooterPIDController = bottomShooterMotor.getClosedLoopController();

  //Configs Sparkmax 2025
  SparkMaxConfig shooterConfig = new SparkMaxConfig();
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
 
  /*
  private SimpleMotorFeedforward topShooterFF = new SimpleMotorFeedforward(
      BoxConstants.kTopShooterS,
      BoxConstants.kTopShooterV
  );
  private SimpleMotorFeedforward bottomShooterFF = new SimpleMotorFeedforward(
      BoxConstants.kBottomShooterS,
      BoxConstants.kBottomShooterV
  );
  */
  // Variables used during SmartDashboard changes
  private double topShooterP     = 0;
  private double bottomShooterP  = 0;
  private double topShooterFF = 0;
  private double bottomShooterFF = 0;
  // shooterMotor variable
  private double shooterSpeed = 0; 
  // Sensor
  private DigitalInput noteSensor = new DigitalInput(BoxConstants.kNoteSensorChannel);

  /** Creates a new Box. */
  public BoxSubsystem() {
    /* Motor Configuration */
shooterConfig
    .inverted(true)
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(40)
    //Enable voltage compensation
    .voltageCompensation(BoxConstants.kShooterMotorNominalVoltage);
shooterConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1000);
shooterConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
//---------------------------------------Intake config-----------------------------------------
intakeConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(20)
    .voltageCompensation(BoxConstants.kIntakeMotorNominalVoltage);
intakeConfig.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
intakeConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(BoxConstants.kTopShooterP, 0, 0, BoxConstants.kBottomShooterFF);
  //-----------------------------------Installing configs---------------------------------------
  topShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  bottomShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Reduce the frequency of the motor position sent to the roboRIO
    //intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    //topShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    //bottomShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535)

    // Set encoders position to 0
    topShooterEncoder.setPosition(0);
    bottomShooterEncoder.setPosition(0);

    // Put Shooter PIDs on SmartDashboard
    SmartDashboard.putNumber("TopShooter P", BoxConstants.kTopShooterP);
    SmartDashboard.putNumber("BottomShooter P", BoxConstants.kBottomShooterP);
    SmartDashboard.putNumber("Top Shooter FF", BoxConstants.kTopShooterFF);
    SmartDashboard.putNumber("Bottom Shooter FF", BoxConstants.kBottomShooterFF);
  }
  
  /**
   * Sets the speed of the intake motor. Takes a double for speed.
   *
   * @param speed     Speed of the motor.
   */
  public Command setIntakeMotorCommand(double speed) {
    return run(() -> intakeMotor.set(speed));
  }

  /**
   * Sets the speed of the intake motor. Takes a double for speed.
   * When the command ends, the intake motor stops.
   *
   * @param speed     Speed of the motor.
   */
  public Command setIntakeMotorCommandThenStop(double speed) {
    return Commands.startEnd(() -> intakeMotor.set(speed), () -> intakeMotor.set(0), this);
  }

  /**
   * Sets the speed of the shooter motor.
   *
   * @param speed     Speed of the motor.
   */
  public Command setShooterMotorCommand(double speed) {
    return run(() -> {
      topShooterMotor.set(speed);
      bottomShooterMotor.set(speed);
    });
  }
  
  /**
   * Sets the speed of the shooter and intake motor. Takes a double for speed.
   *
   * @param shooterSpeed    Speed of the shooter motor.
   * @param intakeSpeed     Speed of the intake motor.
   */
  public Command YeetCommand(double shooterSpeed, double intakeSpeed) {
    return Commands.startEnd(() -> {
      intakeMotor.set(intakeSpeed);
      topShooterMotor.set(shooterSpeed);
      bottomShooterMotor.set(shooterSpeed);
    }, () -> {
      intakeMotor.set(0);
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
    }, this);
  }

  /**
   * Sets the speed of the shooter motor depending on a supplied ArmState.
   *
   * @param position  ArmState Supplier used to set the speed of the shooter motor.
   * @param feeder    A boolean if the feeder motor should also spin. 
   *                  True = feeder motor spins. False = feeder motor does NOT spin.
   */
  public Command setShooterFeederCommand(Supplier<ArmState> position, boolean feeder) {
    return run(() -> {
      switch(position.get()) {
        case SHOOT_SUB:
          shooterSpeed = BoxConstants.kTopSpeakerRPM;
          break;
        case AMP:
          shooterSpeed = BoxConstants.kTopAmpRPM;
          break;
        case IDLE:
          shooterSpeed = 0.0;
          break;
        case SHOOT_HORIZONTAL:
          shooterSpeed = BoxConstants.kTopHorizontalRPM;
          break;
        case SHOOT_N2:
          shooterSpeed = BoxConstants.kTopN2RPM;
          break;
        case TRAP:
          shooterSpeed = BoxConstants.kTopSpeakerRPM;
          break;
        case PASS:
          shooterSpeed = BoxConstants.kTopPassRPM;
          break;
        default:
          shooterSpeed = BoxConstants.kTopDefaultRPM;
          break;
      }

      topShooterPIDController.setReference(shooterSpeed, ControlType.kVelocity);
      bottomShooterPIDController.setReference(shooterSpeed, ControlType.kVelocity);

      if (feeder) {
        intakeMotor.set(BoxConstants.kFeedSpeed);
      }

    });
  }


  // These two commands are ran in auton
  public Command ShootNoteSubwoofer() {
    return setIntakeMotorCommandThenStop(Constants.BoxConstants.kRegurgitateSpeed)
    .withTimeout(.25) 
    .andThen(setShooterMotorCommand(Constants.BoxConstants.kTopSpeakerRPM))
    .withTimeout(BoxConstants.kShooterDelay)
    .andThen(setIntakeMotorCommandThenStop(BoxConstants.kFeedSpeed))
    .withTimeout(2.0)
    .andThen(setShooterMotorCommand(0));
  }
  /*public Command ShootNoteSubwooferNoRegurgitate() {
    return setShooterMotorCommand(0.34)
    .withTimeout(BoxConstants.kShooterDelay)
    .andThen(setIntakeMotorCommandThenStop(BoxConstants.kFeedSpeed))
    .withTimeout(2.0)
    .andThen(setShooterMotorCommand(0));
  }*/
  public Command ShootNoteSubwooferNoRegurgitate() {
    return 
    setShooterFeederCommand(ArmSubsystem::getArmState, false).withTimeout(1.75)
    .andThen(setShooterFeederCommand(ArmSubsystem::getArmState, true).withTimeout(1.0))
    .andThen(stopCommand());
  }

  public Command intakeWithChargedShooter() {
    return
    Commands.startEnd(() -> {
      topShooterPIDController.setReference(BoxConstants.kTopDefaultRPM, ControlType.kVelocity);
      bottomShooterPIDController.setReference(BoxConstants.kTopDefaultRPM, ControlType.kVelocity);
      intakeMotor.set(0.75);
    },
    () -> intakeMotor.set(0), this);
  }

  /**
   * Stops the intake and shooter motor.
   */
  public Command stopCommand() {
    return runOnce(() -> {
      shooterSpeed = 0;
      topShooterMotor.stopMotor();
      bottomShooterMotor.stopMotor();
      intakeMotor.stopMotor();
    });
  }

  /**
   * Returns true if the velocity has approximately reached it's setpoint.
   */
  public boolean isVelocityReached() {
    return (Math.abs(topShooterEncoder.getVelocity() - shooterSpeed) <= BoxConstants.kRPMErrorRange &&
            Math.abs(bottomShooterEncoder.getVelocity() - shooterSpeed) <= BoxConstants.kRPMErrorRange);
  }

  /**
   * Returns true if the note sensor has been triggered.
   */
  public boolean noteSensorTriggered() {
    return !noteSensor.get();
  }

  /*
  public boolean noteSensorUntriggered() {
    return noteSensor.get();
  }
  */

  @Override
  public void periodic() {
    if (topShooterP != SmartDashboard.getNumber("TopShooter P", 0)) {
      topShooterP = SmartDashboard.getNumber("TopShooter P", 0);
      shooterConfig.closedLoop.pid(topShooterP, 0.0, 0.0);
      topShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    if (bottomShooterP != SmartDashboard.getNumber("BottomShooter P", 0)) {
      bottomShooterP = SmartDashboard.getNumber("BottomShooter P", 0);
      shooterConfig.closedLoop.pid(bottomShooterP, 0.0, 0.0);
      bottomShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    if (topShooterFF != SmartDashboard.getNumber("Top Shooter FF", 0)) {
      topShooterFF = SmartDashboard.getNumber("Top Shooter FF", 0);
      shooterConfig.closedLoop.pidf(BoxConstants.kTopShooterP, 0, 0, topShooterFF);
      bottomShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    if (bottomShooterFF != SmartDashboard.getNumber("Bottom Shooter FF", 0)) {
      bottomShooterFF = SmartDashboard.getNumber("Bottom Shooter FF", 0);
      shooterConfig.closedLoop.pidf(BoxConstants.kBottomShooterP, 0, 0, bottomShooterFF);
      bottomShooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    SmartDashboard.putNumber("topShooterMotor Velocity", topShooterEncoder.getVelocity());
    SmartDashboard.putNumber("bottomShooterMotor Velocity", bottomShooterEncoder.getVelocity());
    SmartDashboard.putNumber("shooter Velocity setpoint", shooterSpeed);
    // motor.AppliedOutput() * motor.BusVoltage() gives us our real volts for sparkmax.
    SmartDashboard.putNumber("TopShooterMotorVoltage", topShooterMotor.getAppliedOutput() * topShooterMotor.getBusVoltage());
    SmartDashboard.putNumber("BottomShooterMotorVoltage", bottomShooterMotor.getAppliedOutput() * bottomShooterMotor.getBusVoltage());
    // Volts applied from FF
    //SmartDashboard.putNumber("Shooter FF Volts", BoxConstants.kShooterFF * shooterSpeed);

    SmartDashboard.putBoolean("IR Sensor Value", noteSensorTriggered());

    SmartDashboard.putBoolean("Is Velocity Reached", isVelocityReached());
  }

}