package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ElevatorAndWristMove extends Command {
  private Elevator elevator;
  private Wrist wrist;

  private double elevatorGoalMeters;
  private double wristGoalDegrees;

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
    elevator.setSetpoint(elevatorGoalMeters);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return wrist.atGoal() && elevator.atGoal();
  }
}
