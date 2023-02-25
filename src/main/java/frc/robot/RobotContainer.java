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
import frc.robot.commands.AutonomousDriveCommand;
import frc.robot.commands.ArmGoToHigh;
import frc.robot.commands.ArmGoToHigh2;
import frc.robot.commands.ArmGoToMid;
import frc.robot.commands.ClawGrabCone;
import frc.robot.commands.ClawReset;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendHighGoal;
import frc.robot.commands.ExtenderRetractToZero;
import frc.robot.commands.ManualEncoderCalibration;
import frc.robot.commands.SetTo90;
import frc.robot.commands.OrientCone;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Orienter;
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

    String trajectoryJSON = "paths/ForwardPath.wpilib.json";    

    Trajectory trajectory3 = new Trajectory();

    String chargePad2Json = "paths/ChargePad2.wpilib.json";
    Trajectory chargePad2Trajectory = new Trajectory();

    String chargePad1JSON = "paths/ChargePad1.wpilib.json";
    Trajectory chargePadTragectory = new Trajectory();

  

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();

    private SequentialCommandGroup scoreHighGoal = new SequentialCommandGroup(
      new ClawGrabCone(claw),
      new ArmGoToHigh(arm),
      new InstantCommand(()->extender.extendHighGoal())
    );

    private SequentialCommandGroup scoreHighGoalAuton = new SequentialCommandGroup(
      new ClawGrabCone(claw),
      new ArmGoToHigh(arm),
      new ExtendHighGoal(extender, 2.0)
    );

    private SequentialCommandGroup retractArm = new SequentialCommandGroup(
      new ClawReset(claw),
      new ExtenderRetractToZero(extender),
      new InstantCommand(()->arm.armMoveToZeroPosition()) 
    );

    private SequentialCommandGroup retractArmAuton = new SequentialCommandGroup(
      new ClawReset(claw),
      new ExtenderRetractToZero(extender),
      new InstantCommand(()->arm.armMoveToZeroPosition()) 
    );



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
        limeLightGrid));

      // Configure the button bindings
      ManualEncoderCalibration manualEncoderCalibration = new ManualEncoderCalibration(swerveSubsystem);        
      primaryController.b()

          .onTrue(new OrientCone(orienter, limeLightGrid));

      SmartDashboard.putData(manualEncoderCalibration);
      configureButtonBindings();

      SmartDashboard.putData(new InstantCommand(() -> swerveSubsystem.zeroIntegrator()));

      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        System.out.println("Unable to open trajectory" + ex);
      }

      try {
        Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(chargePad2Json);
        chargePad2Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
      } catch (IOException ex) {
        System.out.println("Unable to open chargePad2 trajectory " + ex);
      }

      try {
        Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(chargePad1JSON);
        chargePadTragectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
      } catch (IOException ex) {
        System.out.print("unable to open charge pad 1" + ex);
      }

      //create auton commands

      ProfiledPIDController thetaController = swerveSubsystem.getThetaController();
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      //Set the odometry to initial pose of the trajectory
      swerveSubsystem.resetOdometry(trajectory3.getInitialPose());
      //Reset all the drive motors of the swervemodules to 0
      swerveSubsystem.resetEncoders();

      // 4. Construct command to follow trajectory
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
              trajectory3,
              swerveSubsystem::getPose,
              DriveConstants.kDriveKinematics,
              swerveSubsystem.getxController(),
              swerveSubsystem.getyController(),
              thetaController,
              swerveSubsystem::setModuleStates,
              swerveSubsystem);
   

        SwerveControllerCommand driveToChargePadCommand = new SwerveControllerCommand(
          chargePadTragectory.concatenate(chargePad2Trajectory),
          swerveSubsystem::getPose,
          DriveConstants.kDriveKinematics,
          swerveSubsystem.getxController(),
          swerveSubsystem.getyController(),
          thetaController,
          swerveSubsystem::setModuleStates,
          swerveSubsystem); 
                
        SequentialCommandGroup autonForwardPath = new SequentialCommandGroup(
                  //new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), 
                  new ManualEncoderCalibration(swerveSubsystem),
                  swerveControllerCommand,  
                  new InstantCommand(() -> swerveSubsystem.stopModules()));     

      AutonomousDriveCommand autonomousDriveCommand = new AutonomousDriveCommand(swerveSubsystem, 6);
      SetTo90 setTo90 = new SetTo90(swerveSubsystem, 0.25);
      
      //right or left
      autonChooser.setDefaultOption("Right or Left", scoreHighGoalAuton
        .andThen(new ArmGoToHigh2(arm))
        .andThen(retractArmAuton)
        .andThen(Commands.parallel(autonForwardPath, new AutonIntakeCommand(intake, 6))));
      //auton Balance
      // autonChooser.setDefaultOption("Right or Left", (new InstantCommand(()-> swerveSubsystem.resetOdometry(trajectory3.getInitialPose()))
      //   .andThen(Commands.parallel(autonForwardPath, new AutonIntakeCommand(intake, 6)))));

      autonChooser.addOption("Auton Balance", driveToChargePadCommand
        .andThen(autonomousDriveCommand).andThen(setTo90));

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
      // secondaryController.pov(270).onTrue(new InstantCommand(()->arm.armHighGoal()))
      //   .onFalse(new InstantCommand(()->arm.stopArm()));
      // secondaryController.pov(90).onTrue(new InstantCommand(()->arm.armMidGoal()))
      //   .onFalse(new InstantCommand(()->arm.stopArm()));

      // secondaryController.pov(45).whileTrue(new InstantCommand(()->arm.armUp()));
      // secondaryController.pov(315).whileTrue(new InstantCommand(()->arm.armUp()));
      // secondaryController.pov(225).whileTrue(new InstantCommand(()->arm.armDown()));
      // secondaryController.pov(135).whileTrue(new InstantCommand(()->arm.armDown()));
      // secondaryController.rightTrigger().whileTrue(new InstantCommand(()->arm.armMidGoal()))
      //   .whileFalse(new InstantCommand(()->arm.stopArm()));
      
      secondaryController.start().onTrue(new InstantCommand(()->arm.armMoveToZeroPosition()));
      //   .onFalse(new InstantCommand(()->arm.stopArm()));


      //Extender Buttons
      secondaryController.b().onTrue(new InstantCommand(()->extender.extend()))
        .onFalse(new InstantCommand(()->extender.stopExtend()));      
      secondaryController.x().onTrue(new InstantCommand(()->extender.retract()))
        .onFalse(new InstantCommand(()->extender.stopExtend()));

      

      //Claw Buttons
      secondaryController.rightBumper().onTrue(new InstantCommand(()->claw.clawRelease()))
        .onFalse(new InstantCommand(()->claw.stopGrab()));
      secondaryController.rightTrigger().onTrue(new InstantCommand(()->claw.clawGrab()))
        .onFalse(new InstantCommand(()->claw.stopGrab()));
      
      
      //Intake Buttons
      primaryController.leftBumper().onTrue(new InstantCommand(()->intake.intakeFeedIn()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      primaryController.rightBumper().onTrue(new InstantCommand(()->intake.intakeFeedOut()))
        .onFalse(new InstantCommand(()->intake.stopIntake()));
      
      

      //Manual Orienter Button
      Trigger orienterTrigger = new Trigger(()->secondaryController.getRightX()<-0.15);
      orienterTrigger.onTrue(new InstantCommand(()->orienter.microwaveManualSpin()))
        .onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      Trigger orienterTrigger2 = new Trigger(()->secondaryController.getRightX()>0.15);
      orienterTrigger2.onTrue(new InstantCommand(()->orienter.microwaveReverseManualSpin()))
        .onFalse(new InstantCommand(()->orienter.stopMicrowave()));
      // if(secondaryController.getRightX()<-0.15){
      //   new InstantCommand(()->orienter.microwaveManualSpin());
      // }else if(secondaryController.getRightX()>0.15){
      //   new InstantCommand(()->orienter.microwaveReverseManualSpin());
      // }else{
      //   new InstantCommand(()->orienter.stopMicrowave());
      // }
      // secondaryController.leftTrigger().whileTrue(new InstantCommand(() -> orienter.microwaveManualSpin()))
      //   .whileFalse(new InstantCommand(()->orienter.stopMicrowave()));
      // secondaryController.rightTrigger().whileTrue(new InstantCommand(() -> orienter.microwaveReverseManualSpin()))
      //   .whileFalse(new InstantCommand(()->orienter.stopMicrowave()));
      

      //Auto command groups
      secondaryController.pov(90).onTrue(new SequentialCommandGroup(
        new InstantCommand(()->extender.gotoDefaultPos()),
        new ClawGrabCone(claw),
        new ArmGoToMid(arm),
        new InstantCommand(()->  
        {
          System.out.print("EXTENDER MID GOAL!@#@!@#$$%^");
          extender.extendMidGoal();})
      ));
      secondaryController.pov(270).onTrue(new SequentialCommandGroup(
        new InstantCommand(()->extender.gotoDefaultPos()),
        new ClawGrabCone(claw),
        new ArmGoToHigh(arm),
        new InstantCommand(()->
        {
          System.out.println("Extender High Goal");
          extender.extendHighGoal();})
      ));


      secondaryController.back().onTrue(scoreHighGoal);
      secondaryController.rightStick().onTrue(retractArm);


      //new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
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
  