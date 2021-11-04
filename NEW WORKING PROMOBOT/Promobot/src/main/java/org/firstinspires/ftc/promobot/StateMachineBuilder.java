package org.firstinspires.ftc.promobot;


/**
 * Created by Collin on 10/7/2017.
 */

// This class is used to build state machines because there are too many options for the usual
// constructor method.

public class StateMachineBuilder {

    private int numberOfStates                          = 2;
    private int initialState                            = 0;
    private int initialDelta                            = 1;
    private StateMachine.LimitBehavior limitBehavior    = StateMachine.LimitBehavior.Wrap;
    private StateMachine.Action buttonAction            = StateMachine.Action.OnlyPress;

    public StateMachineBuilder() { }

    public StateMachine buildStateMachine() {
        return new StateMachine(numberOfStates, initialState, initialDelta, limitBehavior, buttonAction);
    }

    public StateMachineBuilder numberOfStates(int numberOfStates) {
        this.numberOfStates = numberOfStates;
        return this;
    }

    public StateMachineBuilder initialState(int initialState) {
        this.initialState = initialState;
        return this;
    }

    public StateMachineBuilder initialDelta(int initialDelta) {
        this.initialDelta = initialDelta;
        return this;
    }

    public StateMachineBuilder buttonAction(StateMachine.Action buttonAction) {
        this.buttonAction = buttonAction;
        return this;
    }

    public StateMachineBuilder limitBehavior(StateMachine.LimitBehavior limitBehavior) {
        this.limitBehavior = limitBehavior;
        return this;
    }
}
