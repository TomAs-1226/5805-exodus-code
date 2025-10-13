package frc.robot.util;

import java.util.Optional;
import java.util.function.Supplier;

import frc.robot.event.BindableEvent;

/**
 * as the name implies it processes a value thats computed in another thread (core?).. 
 * likely alrdy exists, but o well. more so only used for glorified delays
 * @param <T> the type of value to be computed by the task
 * @author florpy
 */
public class AsyncComputeTask<T> {
    
    /** the value computed by the async task, if it has been */
    private volatile T computedValue;
    /** the task to compute async */
    private final Supplier<T> TASK;
    /** currently running or not */
    private volatile boolean running;
    /** finished or not */
    private volatile boolean complete;
    /** an event thats invoked on computation completion */
    private final BindableEvent<T> COMPLETION_EVENT;

    /**
     * takes a task and creates an asynchronous execution wrapper
     * @param task task to compute async
     */
    public AsyncComputeTask(Supplier<T> task) {
        this.TASK = task;
        this.computedValue = null;
        this.running = false;
        this.complete = false;
        this.COMPLETION_EVENT = new BindableEvent<>(() -> {
            return computedValue;
        });
    }

    /**
     * executes this task asynchronously
     * idk if synchro is needed cuz i dont do multithreaded java
     */
    public synchronized void compute() {
        if (running || complete) return; //dont execute if we're executing
        running = true;
        new Thread(() -> {
            computedValue = TASK.get();
            COMPLETION_EVENT.invoke();
            running = false;
            complete = true;
        }).start(); //defer task
    }

    /**
     * returns an event which is triggered on completion
     * @return event, triggered on completion, which supplies the computed value
     */
    public BindableEvent<T> onCompletion() {
        return this.COMPLETION_EVENT;
    }

    /**
     * returns whether the task completed
     * @return if the task completed or not
     */
    public boolean isDone() {
        return complete;
    }

    /**
     * returns whether the task is running or not
     * @return if the task is running or not
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * returns an optional of the computed value (it may or may not be null)
     * @return optional, maybe containing the computed value
     * @apiNote this method should be avoided; instead, use a callback
     */
    public Optional<T> getComputedValue() {
        return Optional.ofNullable(computedValue);
    }

}
