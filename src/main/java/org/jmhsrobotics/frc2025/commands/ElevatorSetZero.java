package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;

public class ElevatorSetZero extends Command {
  private Elevator elevator;
  private Timer timer = new Timer();

  public ElevatorSetZero(Elevator elevator) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    timer.reset();
    elevator.setVoltage(-0.5);
  }

  @Override
  public void execute() {
    // if the velocity is at or near zero, the timer starts. otherwise it is reset to 0
    if (elevator.getCurrentAmps() > 20) timer.start();
    else timer.reset();
  }

  @Override
  public boolean isFinished() {
    // command finished when the velocity has been at zero for a full second
    return timer.get() > 0.5;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
    if (!interrupted) elevator.setZero();
  }
}
