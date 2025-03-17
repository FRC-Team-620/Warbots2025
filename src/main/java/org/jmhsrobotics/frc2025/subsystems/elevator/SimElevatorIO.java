package org.jmhsrobotics.frc2025.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jmhsrobotics.frc2025.Constants;

public class SimElevatorIO implements ElevatorIO {
  private double goalMeters;
  private boolean isOpenLoop;
  private double controlVoltage;

  ElevatorSim simElevator =
      new ElevatorSim(
          DCMotor.getNeoVortex(2),
          12.0,
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
      double outvolts =
          MathUtil.clamp(
              this.pidController.calculate(simElevator.getPositionMeters()),
              -RobotController.getBatteryVoltage(),
              RobotController.getBatteryVoltage());
      this.simElevator.setInput(outvolts);
    }

    this.simElevator.update(Constants.ksimTimestep);

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

  @Override
  public void setVoltage(double controlVoltage) {
    this.isOpenLoop = true;
    this.controlVoltage = controlVoltage;
    simElevator.setInputVoltage(controlVoltage);
  }

  // I can't find a way to set the encoders to zero, so I am using this as a visual placeholder for
  // when the encoders would be set to zero
  @Override
  public void setZero() {
    simElevator.setState(0.5, 0);
  }
}
