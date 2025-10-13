package frc.robot.event;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * a bindable event, which will invoke all its callbacks when polled and it deems itself valid to run 
 * (some stuff 4 this prbly exists... o well)
 * @param <T> the type that is passed to the callbacks
 * @author florpy
 */
public class BindableEvent<T> {

    /** a list of all registered events (only contains events with conditions) */
    private static final ArrayList<BindableEvent<?>> REGISTERED_EVENTS = new ArrayList<>();
    
    /**
     * an event connection generated from connecting a callback to a bindable event
     * @author florpy
     */
    public class EventConnection {
        
        /** callback to be executed */
        private final BiConsumer<T, EventConnection> CALLBACK;
        /** connection construction source */
        private final BindableEvent<T> SOURCE;
        /** if this event connection is disconnected or not */
        private boolean disconnected = false;

        /**
         * constructs an event connection from the event source and w/ the specified callback function
         * @param event source event
         * @param callback callback function
         */
        private EventConnection(BindableEvent<T> event, BiConsumer<T, EventConnection> callback) {
            this.CALLBACK = callback;
            this.SOURCE = event;
        }

        /**
         * executes this callback
         * @param input supplier of input to process
         */
        private void executeCallback(Supplier<T> input) {
            if (!this.disconnected) {
                this.CALLBACK.accept(input.get(), this);
            }
        }

        /**
         * returns whether this connection is disconnected or not
         * @reuturn disconnected state
         */
        public boolean isDisconnected() {
            return this.disconnected;
        }

        /**
         * disconnects this connection from its parent event
         * @apiNote this only queues this connection for disconnection; upon the source event's poll, this connection will 
         *          only then actually be disconnected
         */
        public void disconnect() {
            this.disconnected = true;
        }

        /**
         * returns the event from which this connection originates
         * @return origin of this connection
         */
        public BindableEvent<T> getSourceEvent() {
            return this.SOURCE;
        }

    }

    /** list of connections currently bound to this event */
    private final ArrayList<EventConnection> CONNECTIONS;
    /** condition that must be met in order to execute callbacks */
    private final BooleanSupplier CONDITION;
    /** the parameter that should be passed into the callbacks on execution */
    private final Supplier<T> CALLBACK_SUPPLIER;
    /** if the event is currently enabled or not */
    private volatile boolean enabled;

    /**
     * creates a new bindable event that will execute its callbacks when the specified condition is met and is polled
     * @param condition condition required
     * @param callbackArg the argument that should be passed into each callback
     */
    public BindableEvent(BooleanSupplier condition, Supplier<T> callbackArg) {
        this.CONNECTIONS = new ArrayList<>();
        this.CONDITION = condition;
        this.CALLBACK_SUPPLIER = callbackArg;
        this.enabled = true;
        REGISTERED_EVENTS.add(this);
    }

    /**
     * creates a new bindable event that can only be invoked manually
     * @param callbackArg argument supplied to all callbacks
     */
    public BindableEvent(Supplier<T> callbackArg) {
        this(() -> false, callbackArg); //a little lazy, but it prbly works ;3
        REGISTERED_EVENTS.remove(this); //extra lazy... but it works aswell!!
    }

    /**
     * registers/binds/connects the specified callback to this event, and returns its connection object
     * @param callback callback function to register
     */
    public EventConnection register(BiConsumer<T, EventConnection> callback) {
        EventConnection connection = new EventConnection(this, callback);
        this.CONNECTIONS.add(connection);
        return connection;
    }

    /**
     * purges connections that are disconnected
     */
    private void updateConnections() {
        this.CONNECTIONS.removeIf(EventConnection::isDisconnected);
    }

    /**
     * disconnects all connections from this event
     */
    public void disconnectAll() {
        for (EventConnection connection : this.CONNECTIONS) {
            connection.disconnect();
        }
        updateConnections();
    }

    /**
     * polls this event, which will execute all callbacks bound to this event if the condition specified returns true
     */
    public void poll() {
        if (this.CONDITION.getAsBoolean()) {
            invoke();
        }
    }

    /**
     * polls all registered events
     */
    public static void pollAllEvents() {
        for (BindableEvent<?> event : REGISTERED_EVENTS) {
            event.poll();
        }
    }

    /**
     * manually invokes this event irregardless if its condition has been met or not
     */
    public void invoke() {
        if (!isEnabled()) return;
        updateConnections();
        for (EventConnection connection : this.CONNECTIONS) {
            connection.executeCallback(this.CALLBACK_SUPPLIER);
        }
    }

    /**
     * returns whether the event is enabled or not
     * @return enabled status
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * sets enabled state
     * @param state new enabled state
     * @apiNote synchronized so u can enabled/disable events in callbacks
     */
    public synchronized void setEnabled(boolean state) {
        this.enabled = state;
    }

    /**
     * removes a registered event (removes from auto poll basically)
     * @param event event
     * @apiNote doesnt do anything if the event isnt {@link #REGISTERED_EVENTS registered}
     */
    public static synchronized void deregister(BindableEvent<?> event) {
        REGISTERED_EVENTS.remove(event);
    }

}
