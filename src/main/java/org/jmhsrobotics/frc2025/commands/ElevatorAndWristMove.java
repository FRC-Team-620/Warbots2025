package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ElevatorAndWristMove extends Command {
  private Elevator elevator;
  private Wrist wrist;

  private double elevatorGoalMeters;
  private double wristGoalDegrees;

  private int controlMode = 2;

  public ElevatorAndWristMove(
      Elevator elevator,
      Wrist wrist,
      double elevatorGoalMeters,
      double wristGoalDegrees,
      int mode) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.elevatorGoalMeters = elevatorGoalMeters;
    this.wristGoalDegrees = wristGoalDegrees;
    this.controlMode = mode;

    addRequirements(elevator);
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    wrist.setSetpoint(wristGoalDegrees);
  }

  @Override
  public void execute() {
    if (wrist.checkWristSafe()) elevator.setSetpoint(elevatorGoalMeters);
  }

  @Override
  public boolean isFinished() {
    // return wrist.atGoal() && elevator.atGoal() && startedElevator;
    // return elevator.atGoal();
    return wrist.atGoal() && elevator.atGoal();
  }
}
