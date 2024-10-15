package frc.robot.telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TelemetryManager {
  private ScheduledExecutorService scheduler;
  private final List<Runnable> tasks = Collections.synchronizedList(new ArrayList<>()); // Synchronized for potential //
                                                                                        // safety.
  private final AtomicBoolean isRunning = new AtomicBoolean(false);
  private final long period;
  private final TimeUnit timeUnit;

  private static TelemetryManager instance;

  // Constructor with configurable period and time unit
  private TelemetryManager(long period, TimeUnit timeUnit) {
    this.period = period;
    this.timeUnit = timeUnit;
    startTasks();
  }

  public synchronized static TelemetryManager getInstance() {
    if (instance == null) {
      instance = new TelemetryManager(20, TimeUnit.MILLISECONDS);
    }
    return instance;
  }

  // Starts task execution
  private void startTasks() {
    if (isRunning.compareAndSet(false, true)) {
      scheduler = Executors.newSingleThreadScheduledExecutor(); // Ensure a new scheduler on restart
      scheduler.scheduleAtFixedRate(this::runTasks, 1000, period, timeUnit);
      System.out.println("Tasks started.");
    } else {
      System.out.println("Tasks are already running.");
    }
  }

  // Runnable method to execute tasks
  private void runTasks() {
    if (tasks.isEmpty()) {
      return; // Skip if there are no tasks.
    }
    synchronized (tasks) { // Ensures thread-safety for tasks access.
      long duration = 0; // Calculate duration
      for (Runnable task : tasks) {
        long startTime = System.currentTimeMillis();
        try {
          task.run();
        } catch (Exception e) {
          handleTaskError(task, e);
        } finally {
          long endTime = System.currentTimeMillis();
          duration = startTime - endTime;
        }
      }
      if (duration > 1) {
        System.out.println("Task ran for " + duration + " ms");
      }
    }
  }

  // Adds a new task (prevents duplicates)
  private void addTask(Runnable task) {
    synchronized (tasks) { // Optional, ensure task is only added by one thread at a time.
      if (!tasks.contains(task)) {
        tasks.add(task);
        System.out.println("Added a new task.");
      } else {
        System.out.println("Task is already in the list.");
      }
    }
  }

  public void addTalonFX(TalonFXMonitor tfxpm) {
    addTask(() -> {
      tfxpm.push();
    });
  }

  public void addCANCoder(CANCoderMonitor ccpm) {
    addTask(() -> {
      ccpm.push();
    });
  }

  public void addSwerveModule(SwerveMonitor spm, Supplier<SwerveDriveState> sds, Supplier<Command> c) {
    addTask(() -> {
      spm.telemeterize(sds, c);
    });
  }

  public void addPigeonIMU(PigeonIMUMonitor ppm) {
    addTask(() -> {
      ppm.push();
    });
  }

  public void addSubsystemCommand(Subsystem subsystem) {
    addTask(() -> {
      var path = subsystem.getName() + "/Command";
      var cmd = subsystem.getCurrentCommand();
      if (cmd != null) {
        SmartDashboard.putString(path, cmd.getName());
      } else {
        SmartDashboard.putString(path, "None");
      }
    });
  }

  // Custom error handler for tasks
  private void handleTaskError(Runnable task, Exception e) {
    System.out.println("Error while executing task: " + e.getMessage());
    // Additional error handling can be placed here, like logging or retrying the
    // task
  }
}
