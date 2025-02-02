package org.jmhsrobotics.frc2025.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimWristIO implements WristIO {

  private double goalDegrees;
  // TODO: move these to contants
  // Gearing is 25:1 then a 24:60
  SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          60.0,
          0.07318977,
          Units.inchesToMeters(24),
          -100,
          100,
          false,
          0,
          new double[0]);
  PIDController pidController = new PIDController(0.1, 0, 0);

  public SimWristIO() {
    SmartDashboard.putData("Wrist PID", pidController);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    double output = this.pidController.calculate(Units.radiansToDegrees(armSim.getAngleRads()));
    this.armSim.setInput(
        MathUtil.clamp(output, -13, 13)); // TODO: Clamp based on current battery bus voltage
    this.armSim.update(0.02); // TODO: use sim timestep constant

    inputs.motorAmps = this.armSim.getCurrentDrawAmps();
    inputs.motorRPM =
        Units.radiansPerSecondToRotationsPerMinute(this.armSim.getVelocityRadPerSec());
    inputs.positionDegrees = Units.radiansToDegrees(armSim.getAngleRads());
  }

  @Override
  public void setPositionDegrees(double angle) {
    this.goalDegrees = angle;
    this.pidController.setSetpoint(angle);
  }

  public double getSetpoint() {
    return goalDegrees;
  }
}
