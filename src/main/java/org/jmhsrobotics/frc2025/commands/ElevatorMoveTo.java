package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class ElevatorMoveTo extends Command {
  private Elevator elevatorSubsystem;
  private double goalMeters;

  private Timer timer = new Timer();

  public ElevatorMoveTo(Elevator elevatorSubsystem, double goalMeters) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.goalMeters = goalMeters;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
    this.elevatorSubsystem.setSetpoint(goalMeters);
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Elevator/Travel Time", timer.get());
  }
}
