package org.jmhsrobotics.frc2025.subsystems.linearActuators;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearActuator extends SubsystemBase {
  private LinearActuatorIO linearActuatorIO;
  private LinearActuatorIOInputsAutoLogged inputs = new LinearActuatorIOInputsAutoLogged();

  private double speedDutyCycle;

  public LinearActuator(LinearActuatorIO linearActuatorIO) {
    this.linearActuatorIO = linearActuatorIO;
  }

  @Override
  public void periodic() {
    linearActuatorIO.updateInputs(inputs);
    linearActuatorIO.set(speedDutyCycle);
  }

  public void set(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
  }
}
