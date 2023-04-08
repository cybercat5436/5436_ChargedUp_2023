// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.AutonomousAutoBalance;
import frc.robot.commands.ArmGoToHighMotionMagic;
import frc.robot.commands.AbsoluteEncoderCalibration;
import frc.robot.commands.ArmGoToMid;
import frc.robot.commands.ExtenderRetractToZero;
import frc.robot.commands.ManualEncoderCalibration;
import frc.robot.commands.MoveToFulcrum;
import frc.robot.commands.SetTo90;
import frc.robot.commands.OrientCone;
import frc.robot.commands.SeekFulcrum;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
        // autonChooser.setDefaultOption("Chargepad 21 point auton", 
        //  util.scoreHighGoal(extender, claw, arm)
        // .andThen(util.retractArm(extender, claw, arm))
        // util.autonDriveCommand("paths/ChargePadForward1.5.wpilib.json", swerveSubsystem)
        // .andThen(util.autonDriveCommand("paths/ChargePadBackward1.5.wpilib.json", swerveSubsystem))
        // .andThen(new MoveToFulcrum(swerveSubsystem))
        // .andThen(new AutonomousAutoBalance(swerveSubsystem, 4.0))
        // .andThen(new SetTo90(swerveSubsystem, 0.25))
        // .andThen(new InstantCommand(() -> System.out.println("Inside the instant command"))));
        // util.autonDriveCommand("paths/exitCommunity.wpilib.json", swerveSubsystem));

        // ****************************************************************
        // Full 21 point auton routine
        Trajectory trajOverChargePad = util.getTrajectory("paths/ChargePadForward1.5.wpilib.json");
        Trajectory trajReverseToFulcrum = util.getTrajectory("paths/ChargePadBackward1.5.wpilib.json");

        autonChooser.setDefaultOption("Chargepad 21 point auton",  
        util.scoreHighGoal(extender, claw, arm)
        .andThen(util.retractArm(extender, claw, arm))
        .andThen(new InstantCommand(() -> swerveSubsystem.resetOdometry(trajOverChargePad.getInitialPose())))
        // .andThen(new ManualEncoderCalibration(swerveSubsystem))
        .andThen(util.getSwerveControllerCommand(trajOverChargePad.concatenate(trajReverseToFulcrum), swerveSubsystem))
        .andThen(new InstantCommand(() -> swerveSubsystem.stopModules()))
        .andThen(new SeekFulcrum(swerveSubsystem))
        .andThen(new MoveToFulcrum(swerveSubsystem))
        .andThen(new AutonomousAutoBalance(swerveSubsystem, 4.0))
        .andThen(new SetTo90(swerveSubsystem, 0.25))
        );

        Trajectory trajForwardPart1 = util.getTrajectory("paths/ChargepadForwardSpeed2.5.wpilib.json");
        Trajectory trajForwardPart2 = util.getTrajectory("paths/ChargepadFowardSpeed1.5.wpilib.json");
        Trajectory trajBackwardPart3 = util.getTrajectory("paths/ChargepadBackwardSpeed2.5.wpilib.json");


        // autonChooser.setDefaultOption("Chargepad 21 point auton V2",  
        // util.scoreHighGoal(extender, claw, arm)
        // .andThen(util.retractArm2(extender, claw, arm))

        // .andThen(new ParallelCommandGroup(

        //   new SequentialCommandGroup(
        //     new ExtenderRetractToZero(extender),
        //     new InstantCommand(()->arm.armMoveToZeroPosition())
        //   ),
        //   new SequentialCommandGroup(
        //     new InstantCommand(() -> swerveSubsystem.resetOdometry(trajForwardPart1.getInitialPose())),
        //     // new ManualEncoderCalibration(swerveSubsystem),
        //     util.getSwerveControllerCommand(trajForwardPart1.concatenate(trajForwardPart2).concatenate(trajBackwardPart3), swerveSubsystem)
        //   )

        // ))

        // .andThen(new InstantCommand(() -> swerveSubsystem.stopModules()))
        // .andThen(new SeekFulcrum(swerveSubsystem))
        // .andThen(new MoveToFulcrum(swerveSubsystem))
        // .andThen(new AutonomousAutoBalance(swerveSubsystem, 4.0))
        // .andThen(new SetTo90(swerveSubsystem, 0.25))
        // );



       



        // ****************************************************************
        // Chargepad deliver and balance 
        // 18 pt auton

        autonChooser.addOption("Chargepad 18 point auton", 
        util.scoreHighGoal(extender, claw, arm)
        .andThen(util.retractArm(extender, claw, arm))
        .andThen(util.autonDriveCommand("paths/OntoChargepad.wpilib.json", swerveSubsystem))
        .andThen(new SeekFulcrum(swerveSubsystem))
        .andThen(new MoveToFulcrum(swerveSubsystem))
        .andThen(new AutonomousAutoBalance(swerveSubsystem, 4.0))
        .andThen(new SetTo90(swerveSubsystem, 0.25))
        .andThen(new InstantCommand(() -> System.out.println("Inside the instant command"))));
        
        
        // //Right path, delivers and drives out of community(Tested)
        // autonChooser.addOption("Right Drive And Deliver", 
        // util.scoreHighGoal(extender, claw, arm)
        // .andThen(util.retractArm(extender, claw, arm))
        // .andThen(Commands.parallel(autonForwardPath, new AutonIntakeCommand(intake, 8))));

        // Trajectory trajRightForward1 = util.getTrajectory("paths/RightPathForward2.5.wpilib.json");
        // Trajectory trajRightForward2 = util.getTrajectory("paths/RightPathForward2.5Part2.wpilib.json");

        // autonChooser.addOption("Right Forward 2.5 TEST", 
        // // util.autonDriveCommand("paths/RightPathForward2.5.wpilib.json", swerveSubsystem)
        // // .andThen(util.autonDriveCommand("paths/RightPathForward2.5Part2.wpilib.json", swerveSubsystem))
        // new InstantCommand(() -> swerveSubsystem.resetOdometry(trajRightForward1.getInitialPose()))
        // .andThen(new ManualEncoderCalibration(swerveSubsystem))
        // .andThen(util.getSwerveControllerCommand(trajRightForward1.concatenate(trajRightForward2), swerveSubsystem))
        // .andThen(new InstantCommand(() -> swerveSubsystem.stopModules()))
        // );



        // ****************************************************************
        // Right Grab and Score  
        // 12 pt auton
        Trajectory traj12Right1 = util.getTrajectory("paths/right-1-1.5.wpilib.json");
        Trajectory traj12Right2 = util.getTrajectory("paths/right-2-1.5.wpilib.json");
        Trajectory traj12Right3 = util.getTrajectory("paths/right-3.wpilib.json");

        autonChooser.addOption("Right Grab-n-Score 12pt",  
        util.scoreHighGoal(extender, claw, arm)
        .andThen(util.retractArm(extender, claw, arm))
        .andThen(new InstantCommand(() -> swerveSubsystem.resetOdometry(traj12Right1.getInitialPose())))
        // .andThen(new ManualEncoderCalibration(swerveSubsystem))
        .andThen(util.getSwerveControllerCommand(traj12Right1, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.intakeFeedIn()))
        .andThen(util.getSwerveControllerCommand(traj12Right2, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.stopIntake()))
        .andThen(util.getSwerveControllerCommand(traj12Right3, swerveSubsystem))
        .andThen(new InstantCommand(() -> swerveSubsystem.stopModules()))
        .andThen(new InstantCommand(() -> intake.intakeFeedOut()).repeatedly().withTimeout(1.5))
        .andThen(new InstantCommand(() -> intake.stopIntake()))
        );


      // ****************************************************************
        // Left Grab and Score  
        // 12 pt auton
        Trajectory traj12Left1 = util.getTrajectory("paths/left-1-1.5.wpilib.json");
        Trajectory traj12Left2 = util.getTrajectory("paths/left-2-1.5.wpilib.json");
        Trajectory traj12Left3 = util.getTrajectory("paths/left-3.wpilib.json");
        Trajectory traj12Left3Offset = util.getTrajectory("paths/left-3-alternate.wpilib.json");

        autonChooser.addOption("(FOR TESTING; DO NOT CHOOSE) Left Grab-n-Score 12pt",  
        util.scoreHighGoal(extender, claw, arm)
        .andThen(util.retractArm(extender, claw, arm))
        .andThen(new InstantCommand(() -> swerveSubsystem.resetOdometry(traj12Left1.getInitialPose())))
        // .andThen(new ManualEncoderCalibration(swerveSubsystem))
        .andThen(util.getSwerveControllerCommand(traj12Left1, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.intakeFeedIn()))
        .andThen(util.getSwerveControllerCommand(traj12Left2, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.stopIntake()))
        .andThen(util.getSwerveControllerCommand(traj12Left3, swerveSubsystem))
        .andThen(new InstantCommand(() -> swerveSubsystem.stopModules()))
        .andThen(new InstantCommand(() -> intake.intakeFeedOut()).repeatedly().withTimeout(1.5))
        .andThen(new InstantCommand(() -> intake.stopIntake()))
        );

      // ****************************************************************
      //   Left Grab and Score with offset
      //   12 pt auton

        autonChooser.addOption("Left Grab-n-Score Offset (FOR TESTING) 12pt",  
        util.scoreHighGoal(extender, claw, arm)
        .andThen(util.retractArm(extender, claw, arm))
        .andThen(new InstantCommand(() -> swerveSubsystem.resetOdometry(traj12Left1.getInitialPose())))
        // .andThen(new ManualEncoderCalibration(swerveSubsystem))
        .andThen(util.getSwerveControllerCommand(traj12Left1, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.intakeFeedIn()))
        .andThen(util.getSwerveControllerCommand(traj12Left2, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.stopIntake()))
        .andThen(util.getSwerveControllerCommand(traj12Left3Offset, swerveSubsystem))
        .andThen(new InstantCommand(() -> swerveSubsystem.stopModules()))
        .andThen(new InstantCommand(() -> intake.intakeFeedOut()).repeatedly().withTimeout(3.0))
        .andThen(new InstantCommand(() -> intake.stopIntake()))
        );

      // ****************************************************************
        // Right Grab   
        // 9 pt auton

        Trajectory traj9Right1 = util.getTrajectory("paths/right-1-1.5.wpilib.json");
        Trajectory traj9Right2 = util.getTrajectory("paths/right-2-1.5.wpilib.json");

        autonChooser.addOption("Right Grab 9pt",  
         util.scoreHighGoal(extender, claw, arm)
        .andThen(util.retractArm(extender, claw, arm))
        .andThen(new InstantCommand(() -> swerveSubsystem.resetOdometry(traj9Right1.getInitialPose())))
        // .andThen(new ManualEncoderCalibration(swerveSubsystem))
        .andThen(util.getSwerveControllerCommand(traj9Right1, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.intakeFeedIn()))
        .andThen(util.getSwerveControllerCommand(traj9Right2, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.stopIntake()))
        );


        // autonChooser.addOption("Right 9pt (TESTING)",  
        //  util.scoreHighGoal(extender, claw, arm)
        // .andThen(util.retractArm(extender, claw, arm))
        // .andThen(new InstantCommand(() -> swerveSubsystem.resetOdometry(traj9Right1.getInitialPose())))
        // // .andThen(new ManualEncoderCalibration(swerveSubsystem))
        // .andThen(util.getSwerveControllerCommand(traj9Right1, swerveSubsystem))
        // // .andThen(new InstantCommand(() -> intake.intakeFeedIn()))
        // // .andThen(util.getSwerveControllerCommand(traj9Right2, swerveSubsystem))
        // // .andThen(new InstantCommand(() -> intake.stopIntake()))
        // );
      // ****************************************************************
        // Left Grab   
        // 9 pt auton

        Trajectory traj9Left1 = util.getTrajectory("paths/left-1-1.5.wpilib.json");
        Trajectory traj9Left2 = util.getTrajectory("paths/left-2-1.5.wpilib.json");

        autonChooser.addOption("Left Grab 9pt",  
        util.scoreHighGoal(extender, claw, arm)
        .andThen(util.retractArm(extender, claw, arm))
        .andThen(new InstantCommand(() -> swerveSubsystem.resetOdometry(traj9Left1.getInitialPose())))
        // .andThen(new ManualEncoderCalibration(swerveSubsystem))
        .andThen(util.getSwerveControllerCommand(traj9Left1, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.intakeFeedIn()))
        .andThen(util.getSwerveControllerCommand(traj9Left2, swerveSubsystem))
        .andThen(new InstantCommand(() -> intake.stopIntake()))
        );


      // ****************************************************************
      // Deliver Only  
      // 6 pt auton
      //Delivers the cone alone
      autonChooser.addOption("Deliver Routine 6pt", 
      util.scoreHighGoal(extender, claw, arm)
      .andThen(util.retractArm(extender, claw, arm)));


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
      //secondaryController.b().and(() -> secondaryController.leftTrigger().getAsBoolean()).onTrue();    
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
  