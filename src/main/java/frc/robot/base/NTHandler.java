package frc.robot.base;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.base.subsystem.Subsystem;

import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A handler class for network table variables; the Robot class uses this to automatically get and set things
 */
public class NTHandler {

    private static final NetworkTable robotTable = NetworkTableInstance.getDefault().getTable("robot");
    private static final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");

    private static HashMap<NetworkTableEntry, Supplier<Object>> setMap;
    private static HashMap<NetworkTableEntry, Consumer<Object>> getMap;

    protected static void init (List<Subsystem> subsystems) {
        setMap = new HashMap<>();
        getMap = new HashMap<>();
        subsystems.forEach(NTHandler::addSubsystem);
    }

    private static void addSubsystem(Subsystem subsystem) {
        subsystem.NTSets().forEach(
            (name, valueSupplier) -> {
                if (name.startsWith("/")) {
                    setMap.put(NetworkTableInstance.getDefault().getEntry(name), valueSupplier);
                } else {
                    setMap.put(robotTable.getEntry(subsystem.name + "/" + name), valueSupplier);
                }
            }
        );
        subsystem.NTGets().forEach(
            (name, valueConsumer) -> {
                if (name.startsWith("/")) {
                    getMap.put(NetworkTableInstance.getDefault().getEntry(name), valueConsumer);
                } else {
                    getMap.put(robotTable.getEntry(subsystem.name + "/" + name), valueConsumer);
                }
            }
        );
    }

    protected static void update() {
        setMap.forEach((entry, valueSupplier) -> entry.setValue(valueSupplier.get()));
        getMap.forEach((entry, valueConsumer) -> valueConsumer.accept(entry.getValue().getValue()));
    }

    public static NetworkTableEntry getRobotEntry(String key) {
        return robotTable.getEntry(key);
    }

    public static NetworkTableEntry getVisionEntry(String key) {
        return visionTable.getEntry(key);
    }

}
