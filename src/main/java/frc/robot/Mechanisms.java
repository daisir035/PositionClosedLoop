package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Mechanisms {
  private final Mechanism2d m_mechanism = new Mechanism2d(1.0, 1.6);
  private final MechanismRoot2d m_carriageRoot = m_mechanism.getRoot("Carriage", 0.5, 0.165);

  public Mechanisms() {
    Color8Bit frameColor = new Color8Bit(Color.kDarkGray);
    m_mechanism.getRoot("Base", 0.15, 0.04)
        .append(new MechanismLigament2d("Baseplate", 0.7, 0.0, 12.0, frameColor));
    m_mechanism.getRoot("Left rail", 0.3, 0.04)
        .append(new MechanismLigament2d("Left upright", 1.4, 90.0, 8.0, frameColor));
    m_mechanism.getRoot("Right rail", 0.7, 0.04)
        .append(new MechanismLigament2d("Right upright", 1.4, 90.0, 8.0, frameColor));
    m_mechanism.getRoot("Top beam", 0.3, 1.44)
        .append(new MechanismLigament2d("Crossbeam", 0.4, 0.0, 8.0, frameColor));

    Color8Bit carriageColor = new Color8Bit(Color.kOrange);
    m_carriageRoot.append(new MechanismLigament2d("Platform", 0.5, 0.0, 14.0, carriageColor));
    m_carriageRoot.append(new MechanismLigament2d("Front", 0.12, 90.0, 6.0,
        new Color8Bit(Color.kYellow)));
    SmartDashboard.putData("Elevator", m_mechanism);
  }

  private final StructPublisher<Pose2d> m_robotPose = NetworkTableInstance.getDefault()
      .getStructTopic("RobotPose", Pose2d.struct)
      .publish();

  private final StructArrayPublisher<Pose3d> m_componentPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("ComponentPoses", Pose3d.struct)
      .publish();

  public void update(double heightMeters, double zeroHeightMeters) {
    m_carriageRoot.setPosition(0.25, heightMeters);
    m_robotPose.set(new Pose2d());
    m_componentPoses.set(new Pose3d[] {
        new Pose3d(0.0, 0.0, heightMeters - zeroHeightMeters, new Rotation3d())
    });
    SmartDashboard.putNumber("Elevator/HeightMeters", heightMeters);
  }

  public void close() {
    m_robotPose.close();
    m_componentPoses.close();
  }
}
