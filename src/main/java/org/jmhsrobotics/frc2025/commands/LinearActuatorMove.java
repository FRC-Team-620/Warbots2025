package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.subsystems.linearActuators.LinearActuator;

public class LinearActuatorMove extends Command {
  protected LinearActuator linearActuator;
  protected double speedDutyCycle;

  protected Timer timer = new Timer();

  public LinearActuatorMove(LinearActuator linearActuator, double speedDutyCycle) {
    this.linearActuator = linearActuator;
    this.speedDutyCycle = speedDutyCycle;
  }

  @Override
  public void initialize() {
    timer.restart();
    linearActuator.set(speedDutyCycle);
  }

  @Override
  public void execute() {
    linearActuator.set(speedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    linearActuator.set(0);
  }
}
