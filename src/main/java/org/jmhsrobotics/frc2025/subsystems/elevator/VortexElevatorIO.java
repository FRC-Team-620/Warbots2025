package org.jmhsrobotics.frc2025.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.util.SparkUtil;

public class VortexElevatorIO implements ElevatorIO {
  private SparkFlex vortexLeft =
      new SparkFlex(Constants.ElevatorConstants.kMotorLeftId, MotorType.kBrushless);
  private SparkFlex vortexRight =
      new SparkFlex(Constants.ElevatorConstants.kMotorRightId, MotorType.kBrushless);
  //private AbsoluteEncoder leftEncoder = vortexLeft.getAbsoluteEncoder();
  //private AbsoluteEncoder rightEncoder = vortexRight.getAbsoluteEncoder();
  private RelativeEncoder leftEncoder = vortexLeft.getEncoder();
  private RelativeEncoder rightEncoder = vortexRight.getEncoder();

  private SparkFlexConfig vortexLeftConfig;
  private SparkFlexConfig vortexRightConfig;
  private SparkClosedLoopController pidController;

  private double goalMeters = 0;

  public VortexElevatorIO() {
    vortexLeftConfig = new SparkFlexConfig();
    vortexLeftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .voltageCompensation(12)
        .inverted(true);

    vortexRightConfig = new SparkFlexConfig();
    vortexRightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .voltageCompensation(12)
        .inverted(false)
        .follow(vortexLeft);

    SparkUtil.tryUntilOk(
        vortexLeft,
        5,
        () ->
            vortexLeft.configure(
                vortexLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        vortexRight,
        5,
        () ->
            vortexRight.configure(
                vortexRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    pidController = vortexLeft.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    SparkUtil.ifOk(
        vortexLeft, vortexLeft::getOutputCurrent, (value) -> inputs.motorAmps[0] = value);
    SparkUtil.ifOk(
        vortexRight, vortexRight::getOutputCurrent, (value) -> inputs.motorAmps[1] = value);

    SparkUtil.ifOk(vortexLeft, leftEncoder::getVelocity, (value) -> inputs.motorRPM[0] = value);
    SparkUtil.ifOk(vortexRight, rightEncoder::getVelocity, (value) -> inputs.motorRPM[1] = value);

    SparkUtil.ifOk(
        vortexLeft, leftEncoder::getPosition, (value) -> inputs.motorPositionMeters[0] = value);
    SparkUtil.ifOk(
        vortexRight, rightEncoder::getPosition, (value) -> inputs.motorPositionMeters[1] = value);
    SparkUtil.ifOk(vortexLeft, vortexLeft::getBusVoltage, (value) -> inputs.motorVolts[0] = value);
    SparkUtil.ifOk(vortexRight, vortexRight::getBusVoltage, (value) -> inputs.motorVolts[1] = value);

    // TODO
    SparkUtil.ifOk(vortexLeft, leftEncoder::getPosition, (value) -> inputs.heightMeters = value);

    pidController.setReference(this.goalMeters, ControlType.kPosition);
  }

  public void setPositionMeters(double positionMeters) {
    this.goalMeters = positionMeters;
  }

  public void setVoltage(double voltage){
    this.vortexLeft.setVoltage(voltage);
    this.vortexRight.setVoltage(voltage);
  }

  public void setZero(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}
