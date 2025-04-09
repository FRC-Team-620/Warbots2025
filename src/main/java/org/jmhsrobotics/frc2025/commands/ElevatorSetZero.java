package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ElevatorSetZero extends Command {
  private Elevator elevator;
  private Wrist wrist;
  private Timer timer = new Timer();

  public ElevatorSetZero(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;

    addRequirements(elevator, wrist);
  }

  @Override
  public void initialize() {
    timer.reset();
    elevator.setVoltage(-1.5);

    if (wrist.getPositionDegrees() > Constants.WristConstants.kSafeAngleDegrees)
      wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
    else wrist.setSetpoint(wrist.getPositionDegrees());
  }

  @Override
  public void execute() {
    // if the velocity is at or near zero, the timer starts. otherwise it is reset to 0
    elevator.setVoltage(-1.5);
    if (elevator.getCurrentAmps() > 50) timer.start();
    else timer.reset();
  }

  @Override
  public boolean isFinished() {
    // command finished when the velocity has been at zero for 1/4 seconds[\]
    return timer.get() > 0.25;
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
    if (!interrupted) elevator.setZero();
    elevator.setSetpoint(0);
  }
}
