package org.jmhsrobotics.frc2025.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;

public class CameraTranformCalibration {

  public static Command getCameraTranformCalibration(Vision vison) {
    StructTopic<Pose3d> blackbirdTopic =
                  NetworkTableInstance.getDefault()
                      .getTable("AdvantageKit")
                      .getStructTopic("RealOutputs/DEBUG/Blackbird/0", Pose3d.struct);
              var blackbirdSub = blackbirdTopic.subscribe(new Pose3d());
              StructTopic<Pose3d> overtureTopic =
              NetworkTableInstance.getDefault()
                  .getTable("AdvantageKit")
                  .getStructTopic("RealOutputs/DEBUG/Overture/0", Pose3d.struct);
          var overtureSub = blackbirdTopic.subscribe(new Pose3d());
    return new InstantCommand(
            () -> {
              
              System.out.println(blackbirdSub.get());
            })
        .ignoringDisable(true);

    // TODO Auto-generated constructor stub
  }

  //    private static Pose3d getPose3dFromNetworkTables(String tableName, String key) {

  //     // Topic asdf = null;
  //     // StructTopic<Pose3d> topic = new StructTopic<>(asdf, Pose3d.struct);
  //     // // Create a StructSubscriber for the Pose3d
  //     // // StructSubscriber<Pose3d> poseSubscriber = new StructSubscriber<>(
  //     // //     tableName + "/" + key, Pose3d.getDefaultInstance());

  //     // // Get the latest Pose3d value
  //     // Pose3d pose = poseSubscriber.get();
  //     // if (pose == null) {
  //     //   throw new IllegalStateException("Pose3d value is not available in NetworkTables.");
  //     // }

  //     // return pose;
  //   }
}
