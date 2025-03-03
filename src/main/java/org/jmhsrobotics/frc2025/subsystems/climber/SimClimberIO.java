package org.jmhsrobotics.frc2025.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.jmhsrobotics.frc2025.Constants;

public class SimClimberIO implements ClimberIO {
  private double motorVolts;

  SingleJointedArmSim climberSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1), 100, .07318977, 0.27, -0.2, 1.2, false, 0, new double[0]);

  public SimClimberIO() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    this.climberSim.setInput(motorVolts);
    this.climberSim.update(Constants.ksimTimestep);

    inputs.motorAmps = this.climberSim.getCurrentDrawAmps();
    inputs.motorRPM = Units.radiansPerSecondToRotationsPerMinute(climberSim.getVelocityRadPerSec());
    inputs.positionDegrees = Units.radiansToDegrees(climberSim.getAngleRads());
  }

  @Override
  public void set(double speedDutyCycle) {
    this.motorVolts =
        -MathUtil.clamp(speedDutyCycle, -0.15, 0.15) * RobotController.getBatteryVoltage();
  }
}
