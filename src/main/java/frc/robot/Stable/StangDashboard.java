package frc.robot.Stable;



import java.util.Arrays;


import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class StangDashboard {
  private static StangDoublePublisher[] numPublishers = new StangDoublePublisher[0];
  private static StangStringPublisher[] stringPublishers = new StangStringPublisher[0];
  private static StangBooleanPublisher[] booleanPublishers = new StangBooleanPublisher[0];
  private static NetworkTableInstance ntInstance;
  private static NetworkTable table;

  public static void init() {
    ntInstance = NetworkTableInstance.getDefault();
    table = ntInstance.getTable("StangDashboard");
  }

  double x = 0;
  double y = 0;

  public static void putNumber(String key, double value) {
    if (Arrays.stream(numPublishers)
        .anyMatch(pub -> key.equals(pub.key))) {
      Arrays.stream(numPublishers)
          .filter(pub -> key.equals(pub.key))
          .findFirst()
          .ifPresent(pub -> pub.value.set(value));
    } else {
      var newPub = new StangDoublePublisher();
      newPub.key = key;
      newPub.value = table.getDoubleTopic(key).publish();
      newPub.value.set(value);
      numPublishers = Arrays.copyOf(numPublishers, numPublishers.length + 1);
      numPublishers[numPublishers.length - 1] = newPub;
    }
  }

  public static void putString(String key, String value) {
    if (Arrays.stream(stringPublishers)
        .anyMatch(pub -> key.equals(pub.key))) {
      Arrays.stream(stringPublishers)
          .filter(pub -> key.equals(pub.key))
          .findFirst()
          .ifPresent(pub -> pub.value.set(value));
    } else {
      var newPub = new StangStringPublisher();
      newPub.key = key;
      newPub.value = table.getStringTopic(key).publish();
      newPub.value.set(value);
      stringPublishers = Arrays.copyOf(stringPublishers, stringPublishers.length + 1);
      stringPublishers[stringPublishers.length - 1] = newPub;
    }
  }

  public static void putBoolean(String key, boolean value) {
    if (Arrays.stream(booleanPublishers)
        .anyMatch(pub -> key.equals(pub.key))) {
      Arrays.stream(booleanPublishers)
          .filter(pub -> key.equals(pub.key))
          .findFirst()
          .ifPresent(pub -> pub.value.set(value));
    } else {
      var newPub = new StangBooleanPublisher();
      newPub.key = key;
      newPub.value = table.getBooleanTopic(key).publish();
      newPub.value.set(value);
      booleanPublishers = Arrays.copyOf(booleanPublishers, booleanPublishers.length + 1);
      booleanPublishers[booleanPublishers.length - 1] = newPub;
    }
  }
}

class StangStringPublisher {
  public String key;
  public StringPublisher value;
}

class StangDoublePublisher {
  public String key;
  public DoublePublisher value;
}
class StangBooleanPublisher {
  public String key;
  public BooleanPublisher value;
}
