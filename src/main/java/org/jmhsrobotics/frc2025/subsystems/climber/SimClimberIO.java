package org.jmhsrobotics.frc2025.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimClimberIO implements ClimberIO {
  private double speedDutyCycle;

  SingleJointedArmSim climberSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1), 100, .07318977, 0.27, -.8, 0, false, 0, new double[0]);

  public SimClimberIO() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    this.climberSim.setInput(speedDutyCycle);
    this.climberSim.update(0.02);

    inputs.motorAmps = this.climberSim.getCurrentDrawAmps();
    inputs.motorRPM = Units.radiansPerSecondToRotationsPerMinute(climberSim.getVelocityRadPerSec());
    inputs.positionDegrees = Units.radiansToDegrees(climberSim.getAngleRads());
  }

  @Override
  public void set(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
  }
}
