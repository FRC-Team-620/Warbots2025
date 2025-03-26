package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ScoreBargeNoWrist extends Command {
  private Elevator elevator;
  private Wrist wrist;
  private Intake intake;

  public boolean hasStarted = false;

  public ScoreBargeNoWrist(Elevator elevator, Wrist wrist, Intake intake) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.intake = intake;

    addRequirements(elevator, wrist, intake);
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
      hasStarted = true;
      elevator.setSetpoint(Constants.ElevatorConstants.kBargeMeters);
    }
    if (hasStarted && elevator.getHeight() > 1.4) {
      intake.set(0.8);
    }
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return elevator.atGoal() && elevator.getSetpoint() == Constants.ElevatorConstants.kBargeMeters;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setSetpoint(0);
    wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
    intake.set(0);
  }
}
