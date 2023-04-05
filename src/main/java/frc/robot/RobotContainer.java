// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonArmDownCommand;
import frc.robot.commands.AutonArmUpCommand;
import frc.robot.commands.AutonGrabCommand;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.AutonReleaseCommand;
import frc.robot.commands.AutonomousAutoBalance;
import frc.robot.commands.ArmGoToHighMotionMagic;
import frc.robot.commands.AbsoluteEncoderCalibration;
import frc.robot.commands.ArmGoToHigh2;
import frc.robot.commands.ArmGoToMid;
import frc.robot.commands.ClawGrabCone;
import frc.robot.commands.ClawReset;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendHighGoal;
import frc.robot.commands.ExtenderRetractToZero;
import frc.robot.commands.ManualEncoderCalibration;
import frc.robot.commands.MoveToFulcrum;
import frc.robot.commands.SetTo90;
import frc.robot.commands.OrientCone;
import frc.robot.commands.SeekFulcrum;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ZeroExtender;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Orienter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public boolean halfSpeed = false;
    private final LimeLight limeLightGrid = new LimeLight("limelight");
    private final LimeLight limeLightOrient = new LimeLight("limelight-orient");
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final Orienter orienter = new Orienter(limeLightOrient);
    private final Claw claw = new Claw();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final Extender extender = new Extender();
    private final CommandXboxController primaryController = new CommandXboxController(1);
    private final CommandXboxController secondaryController = new CommandXboxController(0);
    private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -primaryController.getLeftY(),
        () -> -primaryController.getLeftX(),
        () -> -primaryController.getRightX(),
        () -> !primaryController.start().getAsBoolean(),
        // () -> primaryController.leftBumper().getAsBoolean(),
        () -> primaryController.rightTrigger().getAsBoolean(),
        () -> primaryController.y().getAsBoolean(),
        //() -> primaryController.rightBumper().getAsBoolean(),
        () -> primaryController.x().getAsBoolean(),
        () -> primaryController.getLeftTriggerAxis(),
        () -> primaryController.getRightTriggerAxis(),
        limeLightGrid));

      // Configure the button bindings
      ManualEncoderCalibration manualEncoderCalibration = new ManualEncoderCalibration(swerveSubsystem);  
      AbsoluteEncoderCalibration absoluteEncoderCalibration = new AbsoluteEncoderCalibration(swerveSubsystem);           
      primaryController.b()

          .onTrue(new OrientCone(orienter, limeLightGrid));

      SmartDashboard.putData(manualEncoderCalibration);
      SmartDashboard.putData(absoluteEncoderCalibration);
      configureButtonBindings();

      Utils util = new Utils();

      SmartDashboard.putData(new InstantCommand(() -> swerveSubsystem.zeroIntegrator()));

      SequentialCommandGroup stateMachineAutoBalance = new SequentialCommandGroup(
        new SeekFulcrum(swerveSubsystem),
        new MoveToFulcrum(swerveSubsystem));
      SmartDashboard.putData(stateMachineAutoBalance);

      //Create Trajectory
      Trajectory trajectory = util.getTrajectory("paths/ForwardPathRight.wpilib.json");
      Trajectory trajectory2 = util.getTrajectory("paths/ForwardPathRight1.wpilib.json");
      Trajectory chargePad1Trajectory = util.getTrajectory("paths/ChargePad2mts.wpilib.json");
      //Trajectory goingToReversTrajectory = util.getTrajectory("paths/goingToReverseFulcrum.wpilib.json");
      //Trajectory exitCommunity = util.getTrajectory("paths/exitCommunity.wpilib.json");
      
      //create auton commands
      SwerveControllerCommand autoBalanceTrajectoryCommand = util.getSwerveControllerCommand(chargePad1Trajectory, swerveSubsystem);
      
      
      //Trajectory to Drive to Pad
      SequentialCommandGroup autonDriveToPad = new SequentialCommandGroup(
                    new InstantCommand(() -> swerveSubsystem.resetOdometry(chargePad1Trajectory.getInitialPose())), 
                    new ManualEncoderCalibration(swerveSubsystem),
                    autoBalanceTrajectoryCommand,  
                    new InstantCommand(() -> swerveSubsystem.stopModules()));
  
                
       SequentialCommandGroup autonForwardPath = new SequentialCommandGroup(
                  new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
                  new ManualEncoderCalibration(swerveSubsystem),
                  util.getSwerveControllerCommand(trajectory.concatenate(trajectory2), swerveSubsystem),  
                  new InstantCommand(() -> swerveSubsystem.stopModules())); 

      /**   SequentialCommandGroup twentyOnePointAuton = new SequentialCommandGroup(
          new InstantCommand(() -> swerveSubsystem.resetOdometry(exitCommunity.getInitialPose())),
          new ManualEncoderCalibration(swerveSubsystem),
          util.getSwerveControllerCommand(exitCommunity.concatenate(goingToReversTrajectory), swerveSubsystem)
        );*/
                  

        
        // Command to Auto Balance                      
        AutonomousAutoBalance autonAutoBalance = new AutonomousAutoBalance(swerveSubsystem, 10);
        SetTo90 setTo90 = new SetTo90(swerveSubsystem, 0.25);
        
        
        //for testing
        autonChooser.setDefaultOption("BTI exit for 21 point auton", 
        util.autonDriveCommand("paths/ChargePadForward1.5.wpilib.json", swerveSubsystem)
        .andThen(util.autonDriveCommand("paths/ChargePadBackward1.5.wpilib.json", swerveSubsystem))
        .andThen(new InstantCommand(() -> System.out.println("Inside the instant command"))));
        // util.autonDriveCommand("paths/exitCommunity.wpilib.json", swerveSubsystem));
        
        
        //Right path, delivers and drives out of community(Tested)
        autonChooser.addOption("Right Drive And Deliver", 
        util.scoreHighGoal(extender, claw, arm)
        .andThen(util.retractArm(extender, claw, arm))
        .andThen(Commands.parallel(autonForwardPath, new AutonIntakeCommand(intake, 8))));

        // autonChooser.addOption("Right Forward Backward 2.5", 
        // util.scoreHighGoal(extender, claw, arm)
        // .andThen(util.retractArm(extender, claw, arm))
        // .andThen(util.autonDriveCommand("paths/RightPathForward2.5.wpilib.json", swerveSubsystem))
        // .andThen(util.autonDriveCommand("paths/RightPathBackward2.5.wpilib.json", swerveSubsystem))
        // );

        Trajectory trajRightForward1 = util.getTrajectory("paths/RightPathForward2.5.wpilib.json");
        Trajectory trajRightForward2 = util.getTrajectory("paths/RightPathForward2.5Part2.wpilib.json");

        autonChooser.addOption("Right Forward 2.5 TEST", 
        // util.autonDriveCommand("paths/RightPathForward2.5.wpilib.json", swerveSubsystem)
        // .andThen(util.autonDriveCommand("paths/RightPathForward2.5Part2.wpilib.json", swerveSubsystem))
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajRightForward1.getInitialPose()))
        .andThen(new ManualEncoderCalibration(swerveSubsystem))
        .andThen(util.getSwerveControllerCommand(trajRightForward1.concatenate(trajRightForward2), swerveSubsystem))
        .andThen(new InstantCommand(() -> swerveSubsystem.stopModules()))
        );

        // Full 21 point auton routine
        Trajectory trajOverChargePad = util.getTrajectory("paths/exit-community-v2.wpilib.json");
        Trajectory trajReverseToFulcrum = util.getTrajectory("paths/reverse-to-fulcrum-v2.wpilib.json");

        
  
        autonChooser.addOption("21 point autonV2",  
        // // util.scoreHighGoal(extender, claw, arm)
        // // .andThen(util.retractArm(extender, claw, arm))
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajOverChargePad.getInitialPose()))
        .andThen(new ManualEncoderCalibration(swerveSubsystem))
        .andThen(util.getSwerveControllerCommand(trajOverChargePad.concatenate(trajReverseToFulcrum), swerveSubsystem))
        .andThen(new InstantCommand(() -> swerveSubsystem.stopModules()))
        // // .andThen(new SeekFulcrum(swerveSubsystem))
        // // .andThen(new MoveToFulcrum(swerveSubsystem))
        // // .andThen(new AutonomousAutoBalance(swerveSubsystem, 4.0))
        // // .andThen(new SetTo90(swerveSubsystem, 0.25))
        );


      //Left path, Deliver and drive out of community(Not Tested)
      autonChooser.addOption("Left Drive and Deliver", util.scoreHighGoal(extender, claw, arm)
      .andThen(util.retractArm(extender, claw, arm))
      .andThen(Commands.parallel(util.autonDriveCommand("paths/ForwardPathLeft.wpilib.json", swerveSubsystem)
      , new AutonIntakeCommand(intake, 8))));
    

      //Delivers the cone alone(NOT Tested)
      autonChooser.addOption("Deliver Routine", 
      util.scoreHighGoal(extender, claw, arm)
      .andThen(util.retractArm(extender, claw, arm)));


      autonChooser.addOption("2.5 path ",
      util.scoreHighGoal(extender, claw, arm)
      .andThen(util.retractArm(extender, claw, arm))
      .andThen(util.autonDriveCommand("paths/2.5.wpilib.json", swerveSubsystem))
      .andThen(new SeekFulcrum(swerveSubsystem))
      .andThen(new MoveToFulcrum(swerveSubsystem))
      .andThen(new AutonomousAutoBalance(swerveSubsystem, 8))
      .andThen(new SetTo90(swerveSubsystem, 0.25))
      );

      SmartDashboard.putData(autonChooser);

}
  
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      //Arm Buttons
      secondaryController.pov(0).whileTrue(new InstantCommand(()->arm.armUp()));
      secondaryController.pov(-1).whileTrue(new InstantCommand(()->arm.stopArm()));
      secondaryController.pov(180).whileTrue(new InstantCommand(()->arm.armDown()));

      //Extender Buttons
      secondaryController.b().onTrue(new InstantCommand(()->extender.extend()))
        .onFalse(new InstantCommand(()->extender.stopExtend()));      
      secondaryController.x().onTrue(new InstantCommand(()->extender.retract()))
        .onFalse(new InstantCommand(()->extender.stopExtend()));

      secondaryController.pov(90).onTrue(new InstantCommand(()->orienter.microwaveManualSpin()))
        .onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      secondaryController.pov(270).onTrue(new InstantCommand(()->orienter.microwaveReverseManualSpin()))
        .onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      

      //Claw Buttons
      secondaryController.rightBumper().onTrue(new InstantCommand(()->claw.clawRelease()))
        .onFalse(new InstantCommand(()->claw.stopGrab()));
      secondaryController.leftBumper().onTrue(new InstantCommand(()->claw.clawGrab()))
        .onFalse(new InstantCommand(()->claw.stopGrab()));
     
      
      
      //Intake Buttons
      primaryController.leftBumper().onTrue(new InstantCommand(()->intake.intakeFeedIn()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      primaryController.rightBumper().onTrue(new InstantCommand(()->intake.intakeFeedOut()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      

      //Auto command groups
      secondaryController.start().onTrue(new SequentialCommandGroup(
          new ArmGoToHighMotionMagic(arm),
          new InstantCommand(()->extender.extendHighGoal()) 
      ));
      secondaryController.back().onTrue(new SequentialCommandGroup(
          new ArmGoToMid(arm),
          new InstantCommand(()->extender.extendMidGoal())
      ));
      secondaryController.rightStick().onTrue(new SequentialCommandGroup(
          new ExtenderRetractToZero(extender),
          new InstantCommand(()->arm.armMoveToZeroPosition()) 
      ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

      return autonChooser.getSelected();
              
  }

  }
  