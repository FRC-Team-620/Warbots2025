package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ScoreBarge extends Command {
  private Elevator elevator;
  private Wrist wrist;
  private Intake intake;

  private boolean scoreStarted = false;
  private boolean wristMovementStart = false;

  public ScoreBarge(Elevator elevator, Wrist wrist, Intake intake) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;

    addRequirements(elevator, intake, wrist);
  }

  @Override
  public void initialize() {
    wrist.setSetpoint(Constants.WristConstants.kRotationAlgaeDegrees);
    elevator.setSetpoint(Constants.ElevatorConstants.kProcesserMeters);
  }

  @Override
  public void execute() {
    if (wrist.atGoal()
        && wrist.getSetpoint() == Constants.WristConstants.kRotationAlgaeDegrees
        && elevator.atGoal()
        && elevator.getSetpoint() == Constants.ElevatorConstants.kProcesserMeters) {
      scoreStarted = true;
      elevator.setSetpoint(Constants.ElevatorConstants.kBargeMeters);
    }
    if (this.elevator.getHeight() > 1 && scoreStarted) {
      this.wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
      this.wristMovementStart = true;
    }
    if (this.wrist.getPositionDegrees() < 120 && scoreStarted) {
      this.intake.set(0.8);
    }
  }

  @Override
  public boolean isFinished() {
    return wrist.atGoal() && elevator.atGoal() && wristMovementStart;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setSetpoint(0);
    wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
    intake.set(0);
  }
}
