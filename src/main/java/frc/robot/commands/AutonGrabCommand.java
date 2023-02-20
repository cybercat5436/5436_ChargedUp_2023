package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class AutonGrabCommand extends CommandBase {

private Timer timer;
private double timeLimit = 0.0;
private int encoderCounts = 0;
private Claw claw;

public AutonGrabCommand(Claw claw, int encoderCounts, double time){
    this.encoderCounts = encoderCounts;
    this.timeLimit = time;
    this.claw = claw;
    timer = new Timer();
}
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
     timer.reset();
     timer.start();
     claw.resetClawEncoder();
 }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
     claw.clawGrab();
     
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
     claw.stopGrab();
 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
     
   return (timer.get() > timeLimit ? true : false) || (claw.getClawPosition() >= encoderCounts);
}
}
