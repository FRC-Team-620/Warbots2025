package org.jmhsrobotics.frc2025.subsystems.indexer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class NeoIndexerIO implements IndexerIO {
  private SparkMax motor = new SparkMax(Constants.CAN.kIndexerMotorID, MotorType.kBrushless);
  private AbsoluteEncoder encoder;
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

  private SparkClosedLoopController pidController;

  private double setPointDegrees;

  public NeoIndexerIO() {
    encoderConfig.positionConversionFactor(360).inverted(true);

    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20)
        .voltageCompensation(12)
        .inverted(false)
        .closedLoop
        .pid(
            Constants.IndexerConstants.kP,
            Constants.IndexerConstants.kI,
            Constants.IndexerConstants.kD)
        .outputRange(-0.5, 0.5)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    motorConfig
        .signals
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    motorConfig.absoluteEncoder.apply(encoderConfig);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder = motor.getAbsoluteEncoder();
    pidController = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    SparkUtil.ifOk(motor, encoder::getPosition, (value) -> inputs.positionDegrees = value);
    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.motorAmps = value);

    pidController.setReference(setPointDegrees, ControlType.kPosition);
  }

  @Override
  public void setPositionDegrees(double setPointDegrees) {
    this.setPointDegrees = setPointDegrees;
  }
}
