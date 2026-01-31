package frc.robot.command;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;

import frc.robot.Robot;
import frc.robot.event.BindableEvent;

/**
 * fifo command system 
 * yes i know wpilib has 1 
 * no i dnt rlly care
 * @author florpy
 */
public final class CommandSystem {

    /**
     * debug levels for command system
     * @author florpy
     */
    public enum DebugLevel {
        /** no debugs are printed */
        NONE,
        /** only warnings/errors are printed */
        ISSUES,
        /** all logs are printed */
        ALL
    }

    /** reference to the robot this command system is executing on */
    private final Robot ROBOT;
    /** the current queue of commands to be executed */
    private ArrayList<Command> queue;
    /** debug level set */
    private DebugLevel debugLevel;
    /** an event thats invoked whenever a command executes */
    private final BindableEvent<CommandResult> COMPLETION_EVENT;
    /** the last result recorded from the last command executed */
    private CommandResult lastResult;

    /**
     * constructs a new command system
     * @param robot the robot to bind this command system to
     * @param debugState initial debug state to set to
     */
    public CommandSystem(Robot robot, DebugLevel debugState) {
        this.queue = new ArrayList<>();
        this.ROBOT = robot;
        this.debugLevel = debugState;
        this.lastResult = null;
        if (this.ROBOT == null) issueWarning("robot initialized to null");
        this.COMPLETION_EVENT = new BindableEvent<>(() -> lastResult);
    }

    /**
     * constructs a new command system
     * @param robot robot to bind to
     */
    public CommandSystem(Robot robot) {
        this(robot, DebugLevel.ISSUES);
    }

    /**
     * executes the command system 1 step, or does nothing if nothing is queued
     */
    public void execute() {
        if (queue.isEmpty()) return; //return early
        Command cmd = queue.get(0);
        CommandResult result = cmd.execute(this);
        CommandResult.Type resultType = result.getType();
        //handle debug stuffs
        if (this.debugLevel == DebugLevel.ISSUES || this.debugLevel == DebugLevel.ALL) {
            if (resultType == CommandResult.Type.FAILURE) {
                logDebug(result.getLog(), 2);
            } else {
                logDebug(result.getLog(), 0);
            }
        }
        if (resultType == CommandResult.Type.INTEROP) return; //return cuz its not done
        if (resultType == CommandResult.Type.SUCCESS || resultType == CommandResult.Type.FAILURE) {
            queue.remove(0); //dont need 2 use remove(Object) cuz we know its index 0
            return;
        }
        //handle special yield case
        if (resultType == CommandResult.Type.YIELDED) {
            //swap command queue
            Collections.swap(queue, 0, 1);
        }
    }

    /**
     * returns the robot this command system is bound to
     * @return bound robot of this command system
     */
    public Robot getBoundRobot() {
        return this.ROBOT;
    }

    /**
     * gets the currently set debug level
     * @return current debug level
     */
    public DebugLevel getDebugLevel() {
        return this.debugLevel;
    }

    /**
     * returns whether the debug level is set to anything or not
     * @return if debug level is set
     */
    public boolean isDebugEnabled() {
        return this.debugLevel == DebugLevel.NONE;
    }

    /**
     * sets the debug level
     * @param level new level to set to
     */
    public void setDebugLevel(DebugLevel level) {
        this.debugLevel = level;
    }

    /**
     * pushes a new command to the queue
     * @param cmd new command to queue for execution
     */
    public void queue(Command cmd) {
        this.queue.add(cmd);
    }

    /**
     * logs a debug
     * @param message debug message
     * @param level debug level (0 -> info, 1 -> warning, 2 -> error)
     * @apiNote this is hidden in the backend because it might be too confusing, so the helper methods exist (see below)
     * @see #issueWarning(String)
     * @see #issueError(String)
     * @see #log(String)
     */
    private void logDebug(String message, int level) {
        if (level == 0 && debugLevel != DebugLevel.ALL) return;
        PrintStream out;
        if (level == 2) {
            out = System.err;
        } else {
            out = System.out;
        }
        String log = level == 1 ? "[warn]" : level == 2 ? "[error]" : "[info]";
        //sooo fancy
        log += "//cmd:> " + message;
        out.println(log);
    }

    /**
     * notifies the command system to issue a warning, which will be logged/displayed if the debug level says so
     * @param warning warning message
     */
    public void issueWarning(String warning) {
        if (this.debugLevel == DebugLevel.ALL || this.debugLevel == DebugLevel.ISSUES) {
            logDebug(warning, 1);
        }
    }

    /**
     * notifies the command system to issue an error, which will be logged/displayed if the debug level says so
     * @param error error message
     */
    public void issueError(String error) {
        if (this.debugLevel == DebugLevel.ALL || this.debugLevel == DebugLevel.ISSUES) {
            logDebug(error, 2);
        }
    }

    /**
     * notifies the command system of some info, which will be logged/displayed if the debug level says so
     * @param info info message to log
     */
    public void log(String info) {
        logDebug(info, 0);
    }

    /**
     * returns an event that runs on command completion
     * @return event
     */
    public BindableEvent<CommandResult> getCompletionEvent() {
        return this.COMPLETION_EVENT;
    }
    
}
