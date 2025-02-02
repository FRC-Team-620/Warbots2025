package org.jmhsrobotics.frc2025.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class NeoWristIO implements WristIO {
  private SparkMax motor = new SparkMax(Constants.WristConstants.kMotorId, MotorType.kBrushless);
  private AbsoluteEncoder encoder;

  private SparkMaxConfig motorConfig;

  private SparkClosedLoopController pidController;

  private double goalDegrees;

  public NeoWristIO() {
    motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(false);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder = motor.getAbsoluteEncoder();
    pidController = motor.getClosedLoopController();
  }

  public void updateInputs(WristIOInputs inputs) {
    // getPosition() multiplied by 360 to convert from rotations to degrees
    SparkUtil.sparkStickyFault = false;
    SparkUtil.ifOk(
        motor,
        encoder::getPosition,
        (value) -> inputs.positionDegrees = Units.rotationsToDegrees(value));

    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorAmps = value);
    SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.motorRPM = value);

    pidController.setReference(goalDegrees, ControlType.kPosition);
  }

  public void setPositionDegrees(double positionDegrees) {
    this.goalDegrees = positionDegrees;
  }

  public double getSetpoint() {
    return goalDegrees;
  }
}
