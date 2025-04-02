// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.jmhsrobotics.frc2025.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;
import org.jmhsrobotics.frc2025.Constants;

/** IO implementation for NavX. */
public class GyroIOBoron implements GyroIO {
  private Canandgyro canandgyro;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> xAccelerationTimestampQueue;
  private final Queue<Double> xAccelerationQueue;
  private final Queue<Double> yAccelerationTimestampQueue;
  private final Queue<Double> yAccelerationQueue;

  public GyroIOBoron() {
    canandgyro = new Canandgyro(Constants.CAN.kCanAndGyroID);
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(canandgyro::getYaw);
    xAccelerationTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    xAccelerationQueue =
        SparkOdometryThread.getInstance().registerSignal(canandgyro::getAccelerationX);

    yAccelerationTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yAccelerationQueue =
        SparkOdometryThread.getInstance().registerSignal(canandgyro::getAccelerationY);
  }

  // Check if gyro is calibrated
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = canandgyro.isConnected();
    inputs.calibrated = !canandgyro.isCalibrating();
    inputs.yawPosition = Rotation2d.fromRotations(canandgyro.getYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityYaw());
    inputs.xAcceleration = canandgyro.getAccelerationX();
    inputs.yAcceleration = canandgyro.getAccelerationY();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);

    inputs.odometryYTimestamps =
        xAccelerationTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYAccelerations =
        xAccelerationQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryXTimestamps =
        yAccelerationTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryXAccelerations =
        yAccelerationQueue.stream().mapToDouble((Double value) -> value).toArray();

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
    xAccelerationTimestampQueue.clear();
    xAccelerationQueue.clear();
    yAccelerationTimestampQueue.clear();
    yAccelerationQueue.clear();
  }
}
