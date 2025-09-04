// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

public class ArmSubsystem extends SubsystemBase {
  // Declare and intialize motors and encoders
  private final SparkMax elbowMotorLeader = new SparkMax(ArmConstants.kElbowMotorLeaderID, MotorType.kBrushless);
  private final SparkMax elbowMotorFollower = new SparkMax(ArmConstants.kElbowMotorFollowerID, MotorType.kBrushless);
  private RelativeEncoder elbowEncoder = elbowMotorLeader.getEncoder();
  private final SparkMax wristMotor = new SparkMax(ArmConstants.kWristMotorID, MotorType.kBrushless);
  private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
  // Get the PIDController object for the elbow and wrist
  private SparkClosedLoopController elbowPIDController = elbowMotorLeader.getClosedLoopController();
  private SparkClosedLoopController wristPIDController = wristMotor.getClosedLoopController();
  private PIDController newWristPidController = new PIDController(0.0001, 0, 0);
  // Variables used during SmartDashboard changes
  private double elbowP, elbowI, elbowD, elbowSetPoint = 0;
  private double wristP, wristI, wristD, wristSetPoint = 0;
  // Arm state
  private static ArmState currentPosition = ArmState.IDLE;
  SparkMaxConfig sparky = new SparkMaxConfig();
  SparkMaxConfig sparker = new SparkMaxConfig();
  SparkMaxConfig sparking = new SparkMaxConfig();
  
  
  /* SysID variables and routine */
  /*
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation
  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation
  private final MutableMeasure<Angle> m_angle = MutableMeasure.mutable(Units.Rotations.of(0));
  // Mutable holder for unit-safe linear velocitry values, persisted to avoid reallocation
  private final MutableMeasure<Velocity<Angle>> m_velocity = MutableMeasure.mutable(Units.RPM.of(0));
  // Routine for the arm
  private SysIdRoutine armSysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
        this::motorVoltageControl,
        this::logMotor,
        this)
  );*/

  /* Creates a new Arm. */
  public ArmSubsystem() {
    /* Motor Configuration */

//-------------------------------------------- Elbow Leader   

sparky
    .inverted(true)
    .idleMode(IdleMode.kBrake);
sparky.encoder
    .positionConversionFactor(ArmConstants.kElbowEncoderFactor);
    // .velocityConversionFactor(1000);
sparky.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(ArmConstants.kElbowP, ArmConstants.kElbowI, ArmConstants.kElbowD);
    // elbowPIDController.setFeedbackDevice(elbowEncoder);
     //wristPIDController.setFeedbackDevice(wristEncoder);
//-------------------------------------------- Elbow Follower  
    
 sparker
 .follow(elbowMotorLeader, true);


//-------------------------------------------- Wrist Motor

sparking

    .inverted(false)
    .idleMode(IdleMode.kBrake);
sparking.encoder
    .positionConversionFactor(1);
    // .velocityConversionFactor(1000);
sparking.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(ArmConstants.kWristP, ArmConstants.kWristI, ArmConstants.kWristD);
    // .outputRange(elbowI, elbowD);
    
//--------------------------------------------   

elbowMotorLeader.configure(sparky, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
elbowMotorFollower.configure(sparker, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
wristMotor.configure(sparking, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    // Restore factory defaults of the Spark Max.
  

    // Set motor current limit
    //elbowMotorLeader.setSmartCurrentLimit(40);
    //elbowMotorFollower.setSmartCurrentLimit(40);
    //wristMotor.setSmartCurrentLimit(40);

    // Enable voltage compensation
    //elbowMotorLeader.enableVoltageCompensation(ArmConstants.kElbowMotorNominalVoltage);
    //elbowMotorFollower.enableVoltageCompensation(ArmConstants.kElbowMotorNominalVoltage);
    //wristMotor.enableVoltageCompensation(ArmConstants.kWristMotorNominalVoltage);

    // Reduce data of the follower motor sent to the roboRIO
    //elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    //elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    //elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    //elbowMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);


    // Set encoders position. For the elbow, this is our offset
    elbowEncoder.setPosition(ArmConstants.kElbowAngleOffset);
    wristEncoder.setPosition(0);

  

    // Put Elbow PIDs on SmartDashboard  
    SmartDashboard.putNumber("Elbow P", ArmConstants.kElbowP);
    SmartDashboard.putNumber("Elbow I", ArmConstants.kElbowI);
    SmartDashboard.putNumber("Elbow D", ArmConstants.kElbowD);
    SmartDashboard.putNumber("Elbow Set Point", 0); 
    
    // Put Wrist PIDs on SmartDashboard
    SmartDashboard.putNumber("Wrist P", ArmConstants.kWristP);
    SmartDashboard.putNumber("Wrist I", ArmConstants.kWristI);
    SmartDashboard.putNumber("Wrist D", ArmConstants.kWristD);
    SmartDashboard.putNumber("Wrist Set Point", 0);
  } 


  /**
   * Sets the reference angle of the elbow and wrist.
   * Until the PIDController is given another angle or ControlType, the PID will stay on.
   *
   * @param elbowAngle  The angle the elbow will rotate to and stay at.
   * @param wristAngle  The angle the wrist will rotate to and stay at.
   */
  public Command setArmPIDCommand(ArmState position, boolean stayAtSetpoint) {
    return startEnd(
      // When the command is called, the elbow and wrist PIDController is set and updated on SmartDashboard
      () -> {
        double elbowAngle = 0;
        double wristAngle = 0;
        // Updates currentPosition static var
        currentPosition = position;
        // switch case based on ArmState
        switch(position) {
          case IDLE:
            elbowAngle = ArmConstants.kIdleAngleSP[0];
            wristAngle = ArmConstants.kIdleAngleSP[1];
            break;
          case SOURCE:
            elbowAngle = ArmConstants.kSourceAngleSP[0];
            wristAngle = ArmConstants.kSourceAngleSP[1];
            break;
          case FLOOR:
            elbowAngle = ArmConstants.kFloorAngleSP[0];
            wristAngle = ArmConstants.kFloorAngleSP[1];
            break;
          case AMP:
            elbowAngle = ArmConstants.kAmpAngleSP[0];
            wristAngle = ArmConstants.kAmpAngleSP[1];
            break;
          case SHOOT_SUB:
            elbowAngle = ArmConstants.kSpeakerSubwooferAngleSP[0];
            wristAngle = ArmConstants.kSpeakerSubwooferAngleSP[1];
            break;
          case SHOOT_N2:
            elbowAngle = ArmConstants.kSpeakerN2AngleSP[0];
            wristAngle = ArmConstants.kSpeakerN2AngleSP[1];
            break;
          case TRAP:
            elbowAngle = ArmConstants.kTrapAngleSP[0];
            wristAngle = ArmConstants.kTrapAngleSP[1];
            break;
          case CLIMB_1:
            elbowAngle = ArmConstants.kClimb1AngleSP[0];
            wristAngle = ArmConstants.kClimb1AngleSP[1];
            break;
          case CLIMB_2:
            elbowAngle = ArmConstants.kClimb2AngleSP[0];
            wristAngle = ArmConstants.kClimb2AngleSP[1];
            break;
          case SHOOT_HORIZONTAL:
            elbowAngle = ArmConstants.kHorizontalAngleSP[0];
            wristAngle = ArmConstants.kHorizontalAngleSP[1];
            break;
          case PASS:
            elbowAngle = ArmConstants.kPassAngleSP[0];
            wristAngle = ArmConstants.kPassAngleSP[1];
            break;
          default:
            break;
        }
        elbowPIDController.setReference(elbowAngle, ControlType.kPosition);
        wristPIDController.setReference(wristAngle, ControlType.kPosition);
        SmartDashboard.putNumber("Elbow Set Point", elbowAngle);
        SmartDashboard.putNumber("Wrist Set Point", wristAngle);
      },
      // When the command is interrupted, the elbow and wrist go to their idle position if stayAtSetpoint is false
      () -> {
        if (!stayAtSetpoint) { 
          currentPosition = ArmConstants.ArmState.IDLE;
          elbowPIDController.setReference(ArmConstants.kIdleAngleSP[0], ControlType.kPosition);
          wristPIDController.setReference(ArmConstants.kIdleAngleSP[1], ControlType.kPosition);
        }
      }
    );
  }

  public Command visionArmPIDCommand(DoubleSupplier visionWristAngle) {
    return run(
      () -> {
        currentPosition = ArmConstants.ArmState.SHOOT_SUB;

        elbowPIDController.setReference(ArmConstants.SPEAKER_VISION_ELBOW_SP, ControlType.kPosition);
        wristPIDController.setReference(visionWristAngle.getAsDouble(), ControlType.kPosition);
        
        SmartDashboard.putBoolean("Vision Arm Command", true);
        SmartDashboard.putNumber("Elbow Set Point", ArmConstants.SPEAKER_VISION_ELBOW_SP);
        SmartDashboard.putNumber("Wrist Set Point", visionWristAngle.getAsDouble());
      }
    ).finallyDo(
      () -> {
          SmartDashboard.putBoolean("Vision Arm Command", false);
      }
    );
  }

  public void resetWristEncoder() {
    wristEncoder.setPosition(0);
  };

  /**
   * Sets the elbow motor speed and wrist motor speed in manual mode by giving their PIDControllers
   * a speed in ControlType.kDutyCycle mode.
   * 
   * @param wristSpeed  The speed of the wrist motor from a joystick axis.
   * @param elbowSpeed  The speed of the elbow motor from a joystick axis.
   */
  public Command manualArmCommand(DoubleSupplier wristSpeed, DoubleSupplier elbowSpeed){
    return runOnce(() -> currentPosition = ArmState.MANUAL)
      .andThen(run(() -> {
        wristPIDController.setReference(-wristSpeed.getAsDouble(), ControlType.kDutyCycle);
        elbowPIDController.setReference(elbowSpeed.getAsDouble(), ControlType.kDutyCycle);
    }));
  }

  public Command NewArmCommand( double elbowPosition){
    return 
     Commands.run(() ->{
      if (elbowEncoder.getPosition() < elbowPosition)
      elbowPIDController.setReference(.15, ControlType.kDutyCycle);
      else if (elbowEncoder.getPosition() > elbowPosition) {
        elbowPIDController.setReference(-.15, ControlType.kDutyCycle);

      }


     }, this);
        
  }

  public Command AmpArm(){
    return 
     Commands.run(() ->{
      elbowPIDController.setReference(newWristPidController.calculate(elbowEncoder.getPosition(), 6000), ControlType.kDutyCycle);
      wristPIDController.setReference(ArmConstants.kAmpAngleSP[1], ControlType.kPosition);
    }, this);    
  }

  public Command IntakeArm(){
    return 
     Commands.run(() ->{
      elbowPIDController.setReference(newWristPidController.calculate(elbowEncoder.getPosition(), 300), ControlType.kDutyCycle);
      wristPIDController.setReference(ArmConstants.kFloorAngleSP[1], ControlType.kPosition);
    }, this);    
  }

  public Command HorizontalArm(){
    return 
     Commands.run(() ->{
      elbowPIDController.setReference(newWristPidController.calculate(elbowEncoder.getPosition(), 3000), ControlType.kDutyCycle);
      wristPIDController.setReference(ArmConstants.kHorizontalAngleSP[1], ControlType.kPosition);
    }, this);    
  }

  public Command DefaultStash(){
    return 
     Commands.run(() ->{
      elbowPIDController.setReference(newWristPidController.calculate(elbowEncoder.getPosition(), 150), ControlType.kDutyCycle);
      wristPIDController.setReference(ArmConstants.kIdleAngleSP[1], ControlType.kPosition);
    }, this);    
  }



  /*
  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return armSysIdRoutine.quasistatic(direction);
  }


  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return armSysIdRoutine.dynamic(direction);
  }


  public void motorVoltageControl(Measure<Voltage> volts) {
    elbowMotorLeader.setVoltage(volts.in(Units.Volts));
  }


  public void logMotor(SysIdRoutineLog log) {
    log.motor("elbow-motor")
      .voltage(m_appliedVoltage.mut_replace(elbowMotorLeader.getAppliedOutput() * elbowMotorLeader.getBusVoltage(), Units.Volts))
      .angularPosition(m_angle.mut_replace(elbowEncoder.getPosition(), Units.Rotations))
      .angularVelocity(m_velocity.mut_replace(elbowEncoder.getVelocity(), Units.RPM));
  }
  */


  /**
   * returns our the state of our arm as an enum. 
   * 
   * @return
   */
  public static ArmState getArmState() {
    return currentPosition;
  }
  

  @Override
  public void periodic() {
    // If the elbow PID or setpoint values are different from SmartDashboard, use the new values
    // if (elbowP != SmartDashboard.getNumber("Elbow P", 0)) {
    //   elbowP = SmartDashboard.getNumber("Elbow P", 0);
    //   elbowPIDController.setP(elbowP);
    // }
    // if (elbowI != SmartDashboard.getNumber("Elbow I", 0)) {
    //   elbowI = SmartDashboard.getNumber("Elbow I", 0);
    //   elbowPIDController.setI(elbowI);
    // }
    // if (elbowD != SmartDashboard.getNumber("Elbow D", 0)) {
    //   elbowD = SmartDashboard.getNumber("Elbow D", 0);
    //   elbowPIDController.setD(elbowD);
    // }
    // if (elbowSetPoint != SmartDashboard.getNumber("Elbow Set Point", 0)) {
    //   elbowSetPoint = SmartDashboard.getNumber("Elbow Set Point", 0);
    //   elbowPIDController.setReference(elbowSetPoint, ControlType.kPosition);
    // }

    // // If the wrist PID or setpoint values are different from SmartDashboard, use the new values
    // if (wristP != SmartDashboard.getNumber("Wrist P", 0)) {
    //   wristP = SmartDashboard.getNumber("Wrist P", 0);
    //   wristPIDController.setP(wristP);
    // }
    // if (wristI != SmartDashboard.getNumber("Wrist I", 0)) {
    //   wristI = SmartDashboard.getNumber("Wrist I", 0);
    //   wristPIDController.setI(wristI);
    // }
    // if (wristD != SmartDashboard.getNumber("Wrist D", 0)) {
    //   wristD = SmartDashboard.getNumber("Wrist D", 0);
    //   wristPIDController.setD(wristD);
    // }
    // if (wristSetPoint != SmartDashboard.getNumber("Wrist Set Point", 0)) {
    //   wristSetPoint = SmartDashboard.getNumber("Wrist Set Point", 0);
    //   wristPIDController.setReference(wristSetPoint, ControlType.kPosition);
    // }

    /* 
    elbowPIDController.setReference(elbowSetPoint,
        ControlType.kPosition,
        0,
        ArmConstants.kElbowG * Math.cos(Math.toRadians(elbowEncoder.getPosition()))
        //+ ArmConstants.kElbowS * Math.signum(elbowEncoder.getVelocity())
    );
    */
  
    // Update SmartDashboard with elbow and wrist information
    SmartDashboard.putNumber("Elbow Angular Velocity", elbowEncoder.getVelocity());
    SmartDashboard.putNumber("Elbow Angle", elbowEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Angular Velocity", wristEncoder.getVelocity());
    SmartDashboard.putNumber("Wrist Angle", wristEncoder.getPosition());
    SmartDashboard.putString("NameofEnum", getArmState().toString());
    // motor.AppliedOutput() * motor.BusVoltage() gives us our real volts for sparkmax.
    SmartDashboard.putNumber("ElbowMotorVoltage", elbowMotorLeader.getAppliedOutput() * elbowMotorLeader.getBusVoltage());
  }

}
