package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ScoreBargeWrist extends Command {
  private Wrist wrist;
  private Elevator elevator;
  private Intake intake;

  private boolean readyToScore = false;
  private Timer timer = new Timer();

  public ScoreBargeWrist(Wrist wrist, Elevator elevator, Intake intake) {
    this.wrist = wrist;
    this.elevator = elevator;
    this.intake = intake;

    addRequirements(wrist, elevator, intake);
  }

  @Override
  public void initialize() {
    if (elevator.getSetpoint() == Constants.ElevatorConstants.kBargeMeters && elevator.atGoal())
      readyToScore = true;
    else readyToScore = false;

    timer.reset();
    timer.stop();

    wrist.setSetpoint(Constants.WristConstants.kRotationAlgaeDegrees);
    intake.set(-0.2);
  }

  @Override
  public void execute() {
    if (elevator.getSetpoint() == Constants.ElevatorConstants.kBargeMeters && elevator.atGoal())
      timer.start();
    if (timer.hasElapsed(0.1)) readyToScore = true;

    if (readyToScore) wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
    if (wrist.getPositionDegrees() < 140) {
      intake.set(0.8);
    }
  }

  @Override
  public boolean isFinished() {
    return wrist.atGoal() && wrist.getSetpoint() == Constants.WristConstants.kSafeAngleDegrees;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    elevator.setSetpoint(0);
  }
}
