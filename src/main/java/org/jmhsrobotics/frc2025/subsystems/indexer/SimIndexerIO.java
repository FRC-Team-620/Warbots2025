package org.jmhsrobotics.frc2025.subsystems.climber.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jmhsrobotics.frc2025.Constants;

public class SimIndexerIO implements IndexerIO {
  private double setPointDegrees;
  PIDController pidController = new PIDController(0.02, 0, 0.008);

  SingleJointedArmSim indexerSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          4.0,
          0.07318977,
          Units.inchesToMeters(14),
          0,
          6.28,
          false,
          0,
          new double[0]);

  public SimIndexerIO() {
    SmartDashboard.putData("Indexer PID", pidController);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    double output = this.pidController.calculate(Units.radiansToDegrees(indexerSim.getAngleRads()));

    this.indexerSim.setInput(MathUtil.clamp(output, -13, 13));
    this.indexerSim.update(Constants.ksimTimestep);

    inputs.motorAmps = indexerSim.getCurrentDrawAmps();
    inputs.motorRPM = Units.radiansToDegrees(indexerSim.getVelocityRadPerSec());
    inputs.positionDegrees = Units.radiansToDegrees(indexerSim.getAngleRads());
  }

  @Override
  public void setPositionDegrees(double setPointDegrees) {
    this.setPointDegrees = setPointDegrees;
    this.pidController.setSetpoint(setPointDegrees);
  }
}
