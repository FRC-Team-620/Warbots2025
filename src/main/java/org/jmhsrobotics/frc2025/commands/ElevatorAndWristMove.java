package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ElevatorAndWristMove extends Command {
  private Elevator elevator;
  private Wrist wrist;

  private double elevatorGoalMeters;
  private double wristGoalDegrees;

  private boolean startedElevator = false;

  public ElevatorAndWristMove(
      Elevator elevator, Wrist wrist, double elevatorGoalMeters, double wristGoalDegrees) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.elevatorGoalMeters = elevatorGoalMeters;
    this.wristGoalDegrees = wristGoalDegrees;

    addRequirements(elevator);
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    wrist.setSetpoint(wristGoalDegrees);
  }

  @Override
  public void execute() {
    if (checkWristSafe() && !startedElevator) {
      elevator.setSetpoint(elevatorGoalMeters);
      startedElevator = true;
    }
  }

  /**
   * Ensures that the wrist is in a safe position, and returns true to enable the elevator to start
   * moving Currently just returning true, full functionality coming soon
   *
   * @return
   */
  public boolean checkWristSafe() {
    return true;
  }

  @Override
  public boolean isFinished() {
    // return wrist.atGoal() && elevator.atGoal() && startedElevator;
    // return elevator.atGoal();
    return wrist.atGoal() && elevator.atGoal() && startedElevator;
  }
}
