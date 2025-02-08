package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimElevatorIO implements ElevatorIO {
  private double goalMeters;
  private boolean isOpenLoop;
  private double controlVoltage;

  ElevatorSim simElevator =
      new ElevatorSim(
          DCMotor.getNeoVortex(2),
          10.0,
          Units.lbsToKilograms(20.0),
          Units.inchesToMeters(0.944000),
          0,
          Units.inchesToMeters(78),
          true,
          0,
          new double[0]);
  PIDController pidController = new PIDController(12, 0, 0);

  public SimElevatorIO() {
    SmartDashboard.putData(pidController);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isOpenLoop) {
      this.simElevator.setInputVoltage(controlVoltage);
    } else {
      double output = this.pidController.calculate(simElevator.getPositionMeters());
      this.simElevator.setInput(output);
      this.simElevator.update(0.02);
    }

    inputs.motorAmps =
        new double[] {
          simElevator.getCurrentDrawAmps() / 2.0, simElevator.getCurrentDrawAmps() / 2.0
        };
    inputs.heightMeters = simElevator.getPositionMeters();
    inputs.velocityMPS = simElevator.getVelocityMetersPerSecond();
  }

  @Override
  public void setPositionMeters(double positionMeters) {
    this.isOpenLoop = false;
    this.goalMeters = positionMeters;
    this.pidController.setSetpoint(positionMeters);
  }

  public void setVoltage(double controlVoltage) {
    this.isOpenLoop = true;
    this.controlVoltage = controlVoltage;
    simElevator.setInputVoltage(controlVoltage);
  }

  @Override
  public double getSetpoint() {
    return this.goalMeters;
  }
}
