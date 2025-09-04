// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BoxConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BoxSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband changed to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController P1controller = new CommandXboxController(0);
    private final CommandXboxController P2controller = new CommandXboxController(1);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final BoxSubsystem m_box = new BoxSubsystem();
    private final ArmSubsystem m_arm =  new ArmSubsystem();
    
    // We need to initialize an object of the camera subsystem, we don't have to use it
    //private CameraSubsystem m_CameraSubsystem = new CameraSubsystem(drivetrain);

    public RobotContainer() {

        configureBindings();

        m_box.setDefaultCommand(m_box.stopCommand());
        m_arm.setDefaultCommand(m_arm.DefaultStash());

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        drivetrain.registerTelemetry(logger::telemeterize);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(P1controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(P1controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-P1controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    //.withCenterOfRotation(Translation2d) // FOUND THIS. WILL BE USEFUL FOR DEFENCE SWERVE oR MAYBE SPINNING AROUND AN OBJECT
            )
        );

          // Resets the gyro
   // Resets the gyro
   P1controller.back().onTrue(
    Commands.runOnce(
        () -> drivetrain.seedFieldCentric(),
        drivetrain
    )
);
      
      // Medium speed
      P1controller.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
      drive.withVelocityX(P1controller.getLeftY() * MaxSpeed*Constants.OperatorConstants.kMidModeSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(P1controller.getLeftX() * MaxSpeed*Constants.OperatorConstants.kMidModeSpeed) // Drive left with negative X (left)
          .withRotationalRate(-P1controller.getRightX() * MaxAngularRate*Constants.OperatorConstants.kMidModeSpeed) // Drive counterclockwise with negative X (left)
          //.withCenterOfRotation(Translation2d) // FOUND THIS. WILL BE USEFUL FOR DEFENCE SWERVE oR MAYBE SPINNING AROUND AN OBJECT
  )
      );
    
      // Slow speed
       P1controller.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
       drive.withVelocityX(P1controller.getLeftY() * MaxSpeed*Constants.OperatorConstants.kSlowModeSpeed) // Drive forward with negative Y (forward)
           .withVelocityY(P1controller.getLeftX() * MaxSpeed*Constants.OperatorConstants.kSlowModeSpeed) // Drive left with negative X (left)
           .withRotationalRate(-P1controller.getRightX() * MaxAngularRate*Constants.OperatorConstants.kSlowModeSpeed) // Drive counterclockwise with negative X (left)
           //.withCenterOfRotation(Translation2d) // FOUND THIS. WILL BE USEFUL FOR DEFENCE SWERVE oR MAYBE SPINNING AROUND AN OBJECT
   )
       );
  
  
  
      /* Operator Controls */
  
      // Sets the speed of the shooter motors and starts intake/feed motor
      // When the button is released, the arm goes to idle position and the m_box default command is ran
      // P2controller.leftTrigger().whileTrue(
      //   m_box.setShooterFeederCommand(ArmSubsystem::getArmState, true)
      // ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
  
      //P2controller.rightTrigger().whileTrue(
       //  m_box.setShooterFeederCommand(ArmSubsystem::getArmState, true)
       //).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
       //Intakes note into robot
      //P2controller.rightBumper().whileTrue(m_box.setIntakeMotorCommand(BoxConstants.kIntakeSpeed));
  
      // Regurgitate everything
      P2controller.leftBumper().whileTrue(m_box.YeetCommand(BoxConstants.kRegurgitateSpeed, BoxConstants.kRegurgitateSpeed));
  
      // Smartshoot button, only shoots the note when Velocity is correct and the button is held down.
    //   P2controller.rightTrigger().whileTrue(
    //     Commands.sequence(
    //       Commands.waitUntil(m_box::isVelocityReached),
    //       m_box.setShooterFeederCommand(ArmSubsystem::getArmState, true)
    //     )
    //   ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
    
    //   // Arm set point for climbing
    //   P2controller.button(9).whileTrue(
    //     m_arm.setArmPIDCommand(ArmConstants.ArmState.CLIMB_1, false)
    //   );
  
        // Arm set point for shooting speaker from subwoofer
      //   P2controller.a().whileTrue(
      //   Commands.parallel(
      //     m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_SUB, true),
      //     m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
      //   )
      // ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
  
      // // Arm set point for playing amp
      // P2controller.x().whileTrue(
      //   Commands.parallel(
      //     m_arm.setArmPIDCommand(ArmConstants.ArmState.AMP, true),
      //     m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
      //   )
      // ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
  
      // Idle mode arm set point
      //P2controller.b().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
      P2controller.x().whileTrue(m_arm.AmpArm());

      P2controller.b().whileTrue(m_arm.HorizontalArm());

      P2controller.a().whileTrue(Commands.sequence(
        Commands.parallel(
          m_arm.IntakeArm(),
          m_box.setIntakeMotorCommand(BoxConstants.kIntakeSpeed)
        ).until(m_box::noteSensorTriggered)
      )
    );

      


  
      // Arm set point for shooting podium
    //   P2controller.y().whileTrue(
    //     Commands.parallel(
    //       m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_N2, true),
    //       m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //   ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
  
      // Arm set point for shooting horizontal across the field
      // P2controller.y().whileTrue(
      //   Commands.parallel(
      //     m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_HORIZONTAL, true),
      //     m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
      //   )
      // ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
  
      // Arm set point for shooting trap
      /*m_operatorController.povRight().whileTrue(
        Commands.parallel(
          m_arm.setArmPIDCommand(ArmConstants.ArmState.TRAP, true),
          m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
        )
      ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));*/
  
      // Manual control toggle for arm
      P2controller.start().toggleOnTrue(
          m_arm.manualArmCommand(() -> P2controller.getRightY() * Constants.ArmConstants.kManualSpeed, 
          () -> P2controller.getLeftY() * Constants.ArmConstants.kManualSpeed)
      );
  
      // Smart floor intake with regurgitate?
      P2controller.povDown().whileTrue(
        Commands.sequence(
          Commands.parallel(
            m_arm.setArmPIDCommand(ArmConstants.ArmState.FLOOR, false),
            m_box.setIntakeMotorCommand(BoxConstants.kIntakeSpeed)
          ).until(m_box::noteSensorTriggered)
        )
      );
  
      // Intake from the source
      P2controller.povUp().whileTrue(
        Commands.sequence(
          Commands.parallel(
            m_arm.setArmPIDCommand(ArmConstants.ArmState.SOURCE, false),
            m_box.setIntakeMotorCommand(BoxConstants.kSourceIntakeSpeed)
          ).until(m_box::noteSensorTriggered)
        )
      );
      
    //   P2controller.povRight().whileTrue(
    //     Commands.parallel(
    //         m_arm.setArmPIDCommand(ArmConstants.ArmState.TRAP, true),
    //         m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //       )
    //     ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
  

    //Useful for issues
      // Reset wrist encoder
      //P2controller.back().onTrue(Commands.runOnce(() -> m_arm.resetWristEncoder()));

      

        

       
        
        
       
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // P1controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // P1controller.y().whileTrue(drivetrain.followPath("TestPath")).onFalse(drivetrain.stopCommand());
        // P1controller.x().whileTrue(drivetrain.pathFindThenFollowPath("TestPath")).onFalse(drivetrain.stopCommand());
        //P1controller.b().whileTrue(drivetrain.pathFindToPose(new Pose2d(9, 4, new Rotation2d(0)))).onFalse(drivetrain.stopCommand());

      
      

    

    //     /* Operator Controls */

    //     // Sets the speed of the shooter motors and starts intake/feed motor
    //     // When the button is released, the arm goes to idle position and the m_box default command is ran
    //     P2controller.leftTrigger().whileTrue(
    //     m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, true)
    //     );//.onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, true));

    //     // Intakes note into robot
    //     P2controller.leftBumper().whileTrue(m_BoxSubsystem.setIntakeMotorCommand(BoxConstants.kIntakeSpeed));

    //     // Regurgitate everything
    //     P2controller.rightBumper().whileTrue(m_BoxSubsystem.YeetCommand(BoxConstants.kRegurgitateSpeed, BoxConstants.kRegurgitateSpeed));

    //     // Smartshoot button, only shoots the note when Velocity is correct and the button is held down.
    //     P2controller.rightTrigger().whileTrue(
    //     Commands.sequence(
    //         Commands.waitUntil(m_BoxSubsystem::isVelocityReached),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, true)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
    
    //     // Arm set point for climbing
    //     P2controller.button(9).whileTrue(
    //     m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.CLIMB_1, false)
    //     );

    //     //m_operatorController.button(10).onTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.CLIMB_2, true));
        
    //     // Arm set point for shooting speaker from subwoofer
    //     P2controller.a().whileTrue(
    //     Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.SHOOT_SUB, true),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    //     // Arm set point for playing amp
    //     P2controller.x().whileTrue(
    //     Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.AMP, true),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    //     // Idle mode arm set point
    //     P2controller.b().whileTrue(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    //     // Arm set point for shooting podium
    //     P2controller.povRight().whileTrue(
    //     Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.SHOOT_N2, true),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    //     // Arm set point for shooting horizontal across the field
    //     // Commenting it out to use this button
    //     /*
    //     m_operatorController.povLeft().whileTrue(
    //     Commands.parallel(
    //         m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_HORIZONTAL, true),
    //         m_box.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));
    //     */

    //     // Arm set point for a pass shot
    //     P2controller.povLeft().whileTrue(
    //     Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.PASS, true),
    //         m_BoxSubsystem.setShooterFeederCommand(ArmSubsystem::getArmState, false)
    //     )
    //     ).onFalse(m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));


    //     // Manual control toggle for arm
    //     P2controller.start().toggleOnTrue(
    //         m_ArmSubsystem.manualArmCommand(() -> P2controller.getRightY() * Constants.ArmConstants.kManualSpeed, 
    //         () -> P2controller.getLeftY() * Constants.ArmConstants.kManualSpeed)
    //     );

    //     // Smart floor intake with regurgitate?
    //     P2controller.povDown().whileTrue(
    //     Commands.sequence(
    //         Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.FLOOR, false),
    //         m_BoxSubsystem.setIntakeMotorCommand(BoxConstants.kIntakeSpeed)
    //         ).until(m_BoxSubsystem::noteSensorTriggered)
    //     )
    //     );

    //     // Intake from the source
    //     P2controller.povUp().whileTrue(
    //     Commands.sequence(
    //         Commands.parallel(
    //         m_ArmSubsystem.setArmPIDCommand(ArmConstants.ArmState.SOURCE, false),
    //         m_BoxSubsystem.setIntakeMotorCommand(BoxConstants.kSourceIntakeSpeed)
    //         ).until(m_BoxSubsystem::noteSensorTriggered)
    //     )
    //     );
        

    //     // Reset wrist encoder
    //     P2controller.back().onTrue(Commands.runOnce(() -> m_ArmSubsystem.resetWristEncoder()));
      }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        //return new PathPlannerAuto("Scarlette's Road");
        return autoChooser.getSelected();
    }
}