package org.jmhsrobotics.frc2025.commands;

import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.linearActuators.LinearActuator;

public class LinearActuatorRetract extends LinearActuatorMove {
  public LinearActuatorRetract(LinearActuator linearActuator, double speedDutyCycle) {
    super(linearActuator, speedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > Constants.LinearActuatorConstants.kRetractTimeout;
  }
}
