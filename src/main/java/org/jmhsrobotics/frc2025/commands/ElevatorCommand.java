package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {
  private Elevator elevatorSubsystem;
  private double goalMeters;

  public ElevatorCommand(Elevator elevatorSubsystem, double goalMeters) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.goalMeters = goalMeters;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    this.elevatorSubsystem.setSetpoint(goalMeters);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return elevatorSubsystem.atGoal();
  }
}
