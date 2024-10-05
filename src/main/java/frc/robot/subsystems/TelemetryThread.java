package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class TelemetryThread {
  private ScheduledExecutorService scheduler;
  private final List<Runnable> tasks = Collections.synchronizedList(new ArrayList<>()); // Synchronized for potential //
                                                                                        // safety.
  private final AtomicBoolean isRunning = new AtomicBoolean(false);
  private final long period;
  private final TimeUnit timeUnit;

  // Constructor with configurable period and time unit
  public TelemetryThread(long period, TimeUnit timeUnit) {
    this.period = period;
    this.timeUnit = timeUnit;
  }

  // Starts task execution
  public void startTasks() {
    if (isRunning.compareAndSet(false, true)) {
      scheduler = Executors.newSingleThreadScheduledExecutor(); // Ensure a new scheduler on restart
      scheduler.scheduleAtFixedRate(this::runTasks, 0, period, timeUnit);
      System.out.println("Tasks started.");
    } else {
      System.out.println("Tasks are already running.");
    }
  }

  // Stops task execution gracefully
  public void stopTasks() {
    if (isRunning.compareAndSet(true, false)) {
      scheduler.shutdown();
      try {
        if (!scheduler.awaitTermination(5, TimeUnit.SECONDS)) {
          scheduler.shutdownNow();
        }
      } catch (InterruptedException e) {
        scheduler.shutdownNow();
        Thread.currentThread().interrupt();
      }
      System.out.println("Tasks stopped.");
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
        long startTime = System.currentTimeMillis(); // Capture start time in nanoseconds
        try {
          task.run();
        } catch (Exception e) {
          handleTaskError(task, e);
        } finally {
          long endTime = System.currentTimeMillis(); // Capture end time in nanoseconds
          duration = duration + (startTime - endTime);
        }
      }
      if (duration > 10) {
        System.out.println("Task ran for " + (duration / 1_000_000_000) + " s");
      }
    }
  }

  // Adds a new task (prevents duplicates)
  public void addTask(Runnable task) {
    synchronized (tasks) { // Optional, ensure task is only added by one thread at a time.
      if (!tasks.contains(task)) {
        tasks.add(task);
        System.out.println("Added a new task.");
      } else {
        System.out.println("Task is already in the list.");
      }
    }
  }

  // Removes a task
  public void removeTask(Runnable task) {
    synchronized (tasks) {
      if (tasks.remove(task)) {
        System.out.println("Removed a task.");
      } else {
        System.out.println("Task not found in the list.");
      }
    }
  }

  // Custom error handler for tasks
  private void handleTaskError(Runnable task, Exception e) {
    System.out.println("Error while executing task: " + e.getMessage());
    // Additional error handling can be placed here, like logging or retrying the
    // task
  }
}
