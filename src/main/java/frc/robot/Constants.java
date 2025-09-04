// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;

// /** Add your docs here. */
// public final class Constants {

  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  /* Constants for the swerve chassis */
  public static class SwerveConstants {
    public static final double MAX_SPEED  = 5.6; // maximum m/s for the robot
    public static final double PATHPLANNER_TRANS_KP = 1;
    public static final double LOOP_TIME  = 0.13; // in seconds, 20ms + 110ms spark max velocity lag
  }

  /* Constants for the controllers */
  public static class OperatorConstants {
    public static final int kDriverControllerPort   = 0;
    public static final int kOperatorControllerPort = 1;
    /* Deadbands */
    public static final double LEFT_X_DEADBAND  = 0.04;
    public static final double LEFT_Y_DEADBAND  = 0.04;
    public static final double RIGHT_X_DEADBAND = 0.02;
    public static final double RIGHT_Y_DEADBAND = 0.02;
    /* Speed Modes */
    public static final double kFastModeSpeed = 1;
    public static final double kMidModeSpeed  = 0.6;
    public static final double kSlowModeSpeed = 0.3;
  }

  /* Constants for the arm subsystem */
  

  /* Constants for the box subsystem */
  // public static class BoxConstants{
  //   // Motor IDs
  //   public static final int kTopShooterMotorID = 30;
  //   public static final int kBottomShooterMotorID = 31;
  //   public static final int kIntakeMotorID  = 32;
  //   // Sensor digital input channel
  //   public static final int kNoteSensorChannel = 0;
  //   //Max RPM of shooter motors.
  //   //public static final int kMaxRPM = 4960;
  //   // Nominal Voltage
  //   public static final double kIntakeMotorNominalVoltage  = 10.5;
  //   public static final double kShooterMotorNominalVoltage = 10.5;
  //   // Shooter motor PFF constants
  //   public static final double kTopShooterP = 0.0001;
  //   public static final double kBottomShooterP = 0.0001;
  //   public static final double kTopShooterFF = 0.00021; //(1/kMaxRPM); // same as kV but in percentage instead of volts?
  //   public static final double kBottomShooterFF = 0.000219; //(1/kMaxRPM); // same as kV but in percentage instead of volts?
  //   // Shooter motor speeds
  //   public static final double kTopDefaultRPM    = 3000;
  //   public static final double kBottomDefaultRPM = 3000;
  //   public static final double kTopSpeakerRPM    = 3000;
  //   public static final double kBottomSpeakerRPM = 3000;
  //   public static final double kTopAmpRPM    = 1200;
  //   public static final double kBottomAmpRPM = 1200;
  //   public static final double kTopHorizontalRPM    = 4000;
  //   public static final double kBottomHorizontalRPM = 4000;
  //   public static final double kTopN2RPM = 3700;
  //   public static final double kBottomN2RPM = 3700;
  //   //public static final double kTopYeetRPM    = 2500;
  //   //public static final double kBottomYeetRPM = 2500;
  //   // RPM error range
  //   public static final double kRPMErrorRange = 200; // We are using Math.abs so its +/- kRPMErrorRange
  //   // Intake motor speeds
  //   public static final double kIntakeSpeed      = 1;
  //   public static final double kSourceIntakeSpeed = 0.8;
  //   public static final double kFeedSpeed        = 0.8;
  //   public static final double kRegurgitateSpeed = -0.5;
  //   public static final double kYeetSpeedIntake  = 1;
  //   // Shooter delay
  //   public static final double kShooterDelay     = 1.25;
  //   // Regurgitate time
  //   public static final double kRegurgitateTime  = 0.25;
  //   public static final char Reallyimportantnumber = '7';
  // }

  /*
  public static class LimelightConstants {
    // Wrist offsets
    public static final double wristAngleOffset = 0.0;  // in degrees
    public static final double wristPivotOffsetX = 0.00; // in Meters
    public static final double wristPivotOffsetY = 0.00; // in Meters
    // Speaker height and offset
    public static final double speakerHeight = 0.00;     // height to the middle of the speaker opening
    public static final double speakerOffsetX = 0.22;    // distance to center of speaker opening
  }
  */


  //-----ALL THIS IS NEW WEIRD CONSTANTS ME AND CODY MADE AFTER WORLDS
//}
  public static class DrivetrainConstants {
    // PID Constants
    public static final double PID_XY = 0.55;
    public static final double PID_DEGREE = 0.008;
    public static final double PID_RAD = 6;

    // Feed Forward Constant
    public static final double FEEDFORWARD_CONSTANT = 0.022;
    public static final double FEEDFORWARD_CONSTANT_DEGREE = 0.12;
  }

  //ONLY AFFECTS DRIVE TO POSE, NOT AUTO PATHS
  public static class PathPlannerConstants {
    public static final double MAX_VELOCITY_MPS = 2.450; //5.450
    public static final double MAX_ACCELERATION_MPS = 3.6;
    public static final double MAX_ANGULAR_VELOCITY_RAD = Units.degreesToRadians(360);
    public static final double MAX_ANGULAR_ACCELERATION_RAD = Units.degreesToRadians(862);
    public static final double NOMINAL_VOLTAGE_VOLTS = 11.5;
  }

//   public static class PhotonVisionConstants {
//     //Transform3d from the center of the robot to the camera mount position (ie, robot ➔ camera) in the Robot Coordinate System
//     //The Cameras are mounted on the back of the value so all transform signs are flipped (not rotations). + ➔ -
    
//     // TODO : Fix
//     public static final Transform3d ROBOT_TO_CENTRAL_CAMERA =
//         new Transform3d(Units.inchesToMeters(-12.1), 0, -Units.inchesToMeters(7.55), new Rotation3d(0, Math.PI, 0));
//     public static final Transform3d ROBOT_TO_LEFT_CAMERA =
//     new Transform3d(-0.2545401, -0.1467405,  0.1934088, new Rotation3d(0, Units.degreesToRadians(210), Units.degreesToRadians(-12)));
// }
    
// //   public static final Transform3d ROBOT_TO_RIGHT_CAMERA =
// //   new Transform3d(-0.2545401, 0.1467405, 0.1934088, new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(12)));
// //   public static final Transform3d ROBOT_TO_LEFT_CAMERA =
// //   new Transform3d(-0.2545401, -0.1467405,  0.1934088, new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(-12)));

//   public static class PoseConstants {

//     public static final Pose2d REEF_SIDE_POSE_AB = new Pose2d(3, 4, new Rotation2d(0)); //WORKS ON RED
//     public static final Pose2d REEF_SIDE_POSE_CD = new Pose2d(3.75, 2.75, new Rotation2d(Units.degreesToRadians(240))); // DOESN'T WORK ON RED
//     public static final Pose2d REEF_SIDE_POSE_EF = new Pose2d(5.25, 2.75, new Rotation2d(Units.degreesToRadians(300))); // DOESN'T WORK ON RED
//     public static final Pose2d REEF_SIDE_POSE_GH = new Pose2d(5.9, 4, new Rotation2d(Units.degreesToRadians(0))); //WORKS ON RED
//     public static final Pose2d REEF_SIDE_POSE_IJ = new Pose2d(5.4, 5.5, new Rotation2d(Units.degreesToRadians(60))); // DOESN'T WORK ON RED
//     public static final Pose2d REEF_SIDE_POSE_KL = new Pose2d(3.6, 5.5, new Rotation2d(Units.degreesToRadians(120))); // DOESN'T WORK ON RED

//     public static final Pose2d[] BLUE_REEF_SIDE_POSES = {
//       REEF_SIDE_POSE_AB, REEF_SIDE_POSE_CD, REEF_SIDE_POSE_EF,
//       REEF_SIDE_POSE_GH, REEF_SIDE_POSE_IJ, REEF_SIDE_POSE_KL
//     };

//     public static final double RED_TRANSFORMATION_X = 8.5;

//     public static final Pose2d[] RED_REEF_SIDE_POSES = {
//       new Pose2d(REEF_SIDE_POSE_GH.getX() + 8.5,REEF_SIDE_POSE_GH.getY(), REEF_SIDE_POSE_GH.getRotation()),
//       new Pose2d(REEF_SIDE_POSE_IJ.getX() + 8.5,REEF_SIDE_POSE_IJ.getY(), REEF_SIDE_POSE_IJ.getRotation()),
//       new Pose2d(REEF_SIDE_POSE_KL.getX() + 8.5,REEF_SIDE_POSE_KL.getY(), REEF_SIDE_POSE_KL.getRotation()),
//       new Pose2d(REEF_SIDE_POSE_AB.getX() + 8.5,REEF_SIDE_POSE_AB.getY(), REEF_SIDE_POSE_AB.getRotation()),
//       new Pose2d(REEF_SIDE_POSE_CD.getX() + 8.5,REEF_SIDE_POSE_CD.getY(), REEF_SIDE_POSE_CD.getRotation()),
//       new Pose2d(REEF_SIDE_POSE_EF.getX() + 8.5,REEF_SIDE_POSE_EF.getY(), REEF_SIDE_POSE_EF.getRotation())
//     };

//     public static final Pose2d REEF_POSE_A = new Pose2d(3.2, 4.1, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_B = new Pose2d(3.2, 3.87, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_C = new Pose2d(3.6, 3.1, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_D = new Pose2d(4.1, 2.87, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_E = new Pose2d(5.1, 2.75, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_F = new Pose2d(5.4, 2.9, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_G = new Pose2d(6, 3.8, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_H = new Pose2d(6, 4.2, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_I = new Pose2d(5.4, 5.15, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_J = new Pose2d(5.1, 5.35, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_K = new Pose2d(4, 5.4, new Rotation2d(0));
//     public static final Pose2d REEF_POSE_L = new Pose2d(3.6, 5.2, new Rotation2d(0));

//     public static final Pose2d[] BLUE_REEF_POSES = {
//       REEF_POSE_A, REEF_POSE_B, REEF_POSE_C, REEF_POSE_D, REEF_POSE_E, REEF_POSE_F, 
//       REEF_POSE_G, REEF_POSE_H, REEF_POSE_I, REEF_POSE_J, REEF_POSE_K, REEF_POSE_L
//     };

//     public static final Transform2d RED_TRANSFORMATION = new Transform2d(8.5, 0, new Rotation2d(0));
    
//     public static final Pose2d[] RED_REEF_POSES = {
//       REEF_POSE_G.plus(RED_TRANSFORMATION), REEF_POSE_H.plus(RED_TRANSFORMATION), REEF_POSE_I.plus(RED_TRANSFORMATION),
//       REEF_POSE_J.plus(RED_TRANSFORMATION), REEF_POSE_K.plus(RED_TRANSFORMATION), REEF_POSE_L.plus(RED_TRANSFORMATION),
//       REEF_POSE_A.plus(RED_TRANSFORMATION), REEF_POSE_B.plus(RED_TRANSFORMATION), REEF_POSE_C.plus(RED_TRANSFORMATION),
//       REEF_POSE_D.plus(RED_TRANSFORMATION), REEF_POSE_E.plus(RED_TRANSFORMATION), REEF_POSE_F.plus(RED_TRANSFORMATION)
//     };

//     public static final double BLUE_CORAL_STATION_DEG = 126;
//     public static final double RED_TRANSFORMATION_STATION_X = 15.1;
//     public static final double RED_CORAL_STATION_DEG = 54;

//     public static final Pose2d CORAL_STATION_LEFT_BLUE_POSE = new Pose2d(1.175, 6.926, new Rotation2d(Units.degreesToRadians(BLUE_CORAL_STATION_DEG)));
//     public static final Pose2d CORAL_STATION_RIGHT_BLUE_POSE = new Pose2d(1.175, 1.1, new Rotation2d(Units.degreesToRadians(-BLUE_CORAL_STATION_DEG)));

//     public static final Pose2d CORAL_STATION_LEFT_RED_POSE = new Pose2d(
//       CORAL_STATION_RIGHT_BLUE_POSE.getX() + RED_TRANSFORMATION_STATION_X,
//       CORAL_STATION_RIGHT_BLUE_POSE.getY(),
//       //new Rotation2d() 
//       new Rotation2d(Units.degreesToRadians(-RED_CORAL_STATION_DEG))
//     );
//     public static final Pose2d CORAL_STATION_RIGHT_RED_POSE = new Pose2d(
//       CORAL_STATION_LEFT_BLUE_POSE.getX() + RED_TRANSFORMATION_STATION_X,
//       CORAL_STATION_LEFT_BLUE_POSE.getY(),
//       new Rotation2d(Units.degreesToRadians(RED_CORAL_STATION_DEG)) //new Rotation2d(Units.degreesToRadians(CORAL_STATION_LEFT_BLUE_POSE.getRotation().getDegrees() - RED_TRANSFORMATION_STATION_DEG))
//     );

    
//   }
  

//   // // WE STOLE LITERALLY ALL OF THIS, BRUH
   
   /* Constants for the arm subsystem */
   public static class ArmConstants{
    // All of our PID Postions for the arm
    public static enum ArmState {IDLE, FLOOR, SOURCE, AMP, SHOOT_SUB, SHOOT_N2, SHOOT_HORIZONTAL, CLIMB_1, CLIMB_2, TRAP, PASS, MANUAL};
    // Motor IDs
    public static final int kElbowMotorLeaderID   = 20;
    public static final int kElbowMotorFollowerID = 21;
    public static final int kWristMotorID         = 22;
    // NominalVoltage
    public static final double kElbowMotorNominalVoltage = 10.5;
    public static final double kWristMotorNominalVoltage = 10.5;
    // Encoder Conversion Factor
    public static final double kElbowEncoderFactor = 180;
    // Elbow and wrist PID + PID max speed
    public static final double kElbowP        = 0.00025;//0.025
    public static final double kElbowI        = 0.0;
    public static final double kElbowD        = 0.01;
    public static final double kElbowMinSpeed = -0.6;
    public static final double kElbowMaxSpeed = 0.6;
    public static final double kWristP        = 0.1;
    public static final double kWristI        = 0.0;
    public static final double kWristD        = 0.05;
    public static final double kWristMinSpeed = -0.6;
    public static final double kWristMaxSpeed = 0.6;
    // Elbow FF gravity constant
    //public static final double kElbowS = 0;
    public static final double kElbowG = 0;
    // Elbow offset. The angle should be 0 degrees when parallel
    public static final double kElbowAngleOffset = 0.0;

    // Setpoints for the arm subsystem
    // {Elbow Angle, Wrist Angle} SP = SetPoint
    public static final double[] kIdleAngleSP             = {-0.25, 0};
    public static final double[] kFloorAngleSP            = {0, -34.5};//-34.5
    public static final double[] kSourceAngleSP           = {-48.95, -20.88 }; //-60.51, 21.57 -33.31, 9.285
    public static final double[] kAmpAngleSP              = {-115.44, -37.93}; //-113.7, 31.64
    public static final double[] kSpeakerSubwooferAngleSP = {-9, -38}; //31.64
    public static final double[] kSpeakerN2AngleSP        = {9, -26}; // we need to set these values to be not false
    public static final double[] kHorizontalAngleSP       = {-33.4, -28.5};
    public static final double[] kClimb1AngleSP           = {-98, -34};
    public static final double[] kClimb2AngleSP           = {-43.6, -71.1};
    public static final double[] kTrapAngleSP             = {-85, -51}; // Shoot Subwoofer with intake within bumper bounderies
    public static final double[] kPassAngleSP             = {-13.33, -8.31};

    public static final double SPEAKER_VISION_ELBOW_SP = 0; //-9
    public static final double SPEAKER_1_METER_WRIST_SP = -30.25;
    public static final double SPEAKER_2_METER_WRIST_SP = -28;
    public static final double SPEAKER_3_METER_WRIST_SP = -26.75;
    public static final double SPEAKER_4_METER_WRIST_SP = -25.75;
    public static final double SPEAKER_5_METER_WRIST_SP = -25;

    // This should be the same number as the highest SPEAKER_?_METER_WRIST_SP
    public static final double MAX_SPEAKER_VISION_METERS = 5;

    public static final double kManualSpeed = 0.8;
  }

  // // /* Constants for the box subsystem */
  public static class BoxConstants{
    // Motor IDs
    public static final int kTopShooterMotorID = 30;
    public static final int kBottomShooterMotorID = 31;
    public static final int kIntakeMotorID  = 32;
    // Sensor digital input channel
    public static final int kNoteSensorChannel = 0;
    //Max RPM of shooter motors.
    //public static final int kMaxRPM = 4960;
    // Nominal Voltage
    public static final double kIntakeMotorNominalVoltage  = 10.5;
    public static final double kShooterMotorNominalVoltage = 10.5;
    // Shooter motor PFF constants
    public static final double kTopShooterP = 0.0002;
    public static final double kBottomShooterP = 0.0002;
    public static final double kTopShooterFF = 0.000207; //(1/kMaxRPM); // same as kV but in percentage instead of volts?
    public static final double kBottomShooterFF = 0.000237; //(1/kMaxRPM); // same as kV but in percentage instead of volts?
    // Shooter motor speeds
    public static final double kTopDefaultRPM    = 3500;
    public static final double kTopSpeakerRPM    = 3500;
    public static final double kTopAmpRPM        = 1200;
    public static final double kTopHorizontalRPM = 4000;
    public static final double kTopN2RPM         = 3700;
    public static final double kTopPassRPM       = 3500;
    //public static final double kTopYeetRPM    = 2500;
    //public static final double kBottomYeetRPM = 2500;
    // RPM error range
    public static final double kRPMErrorRange = 200; // We are using Math.abs so its +/- kRPMErrorRange
    // Intake motor speeds
    public static final double kIntakeSpeed      = -1;
    public static final double kSourceIntakeSpeed = -0.8;
    public static final double kFeedSpeed        = -0.8;
    public static final double kRegurgitateSpeed = 0.5;
    public static final double kYeetSpeedIntake  = -1;
    // Shooter delay
    public static final double kShooterDelay     = 1.25;
    // Regurgitate time
    public static final double kRegurgitateTime  = 0.25;
  }
 
}


