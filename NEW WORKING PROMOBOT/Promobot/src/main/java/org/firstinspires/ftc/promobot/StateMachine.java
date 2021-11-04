package org.firstinspires.ftc.promobot;

/**
 * Created by Collin on 10/7/2017.
 */

public class StateMachine {

    // What button action causes the state machine to change
    public enum Action {PressAndRelease, OnlyPress, OnlyRelease};

    // Behavior when the state reaches a limit:
    // - Wrap = wrap around, i.e. n-3, n-2, n-1, 0, 1, ..
    // - Reverse = reverse count direction, i.e. n-3, n-2, n-1, n-2, n-3, ..
    // - Stop = stops at limit value, i.e n-3, n-2, n-1, n-1, n-1, ..
    public enum LimitBehavior {Wrap, Reverse, Stop}

    // current state value
    private int currentState;

    // number of states, value from 0 to maxState-1
    private int maxState;

    // increment / decrement value for the state machine value
    private int delta;

    // Which actions cause the state to change.
    private Action action         = Action.OnlyPress;

    // What happens when the state reaches the minimum or maximum value
    private LimitBehavior limit   = LimitBehavior.Wrap;

    // last button state
    private boolean lastButton1Pressed    = false;
    private boolean lastButton2Pressed    = false;

    //*******  Methods  **********
    public StateMachine(int numberOfStates, int initialState, int initialDelta, LimitBehavior limitBehavior, Action buttonAction){
        maxState     = numberOfStates;
        currentState = initialState;
        delta        = initialDelta;
        limit        = limitBehavior;
        action       = buttonAction;
    }

    public boolean stateChange(boolean buttonPressed){
        boolean ret = isStateChanged(1, buttonPressed);

        if (ret == true){
            updateState(true);
        }
        return ret;
    }

    public boolean stateChange(boolean button1Pressed, boolean button2Pressed) {
        boolean ret = false;
        boolean ret1 = isStateChanged(1, button1Pressed);
        boolean ret2 = isStateChanged(2, button2Pressed);

        if (ret1 && !ret2) {
            updateState(true);
            ret = true;
        } else if (!ret1 && ret2) {
            updateState(false);
            ret = true;
        }
        return ret;
    }

    public int getState() {
        return currentState;
    }

    public void setState(int state) { currentState = state; }

    // Returns true is state changed based on button press and state machine action selection
    private boolean isStateChanged(int button, boolean buttonPressed) {
        boolean ret = false;
        boolean lastButtonPressed = false;

        // Get the last state of the button
        if (button == 1) {
            lastButtonPressed = lastButton1Pressed;
        } else if (button == 2) {
            lastButtonPressed = lastButton2Pressed;
        } else {
            return false; // unknown button is not tested
        }

        if (action == Action.PressAndRelease || action == Action.OnlyPress) {
            ret |= buttonPressed == true && lastButtonPressed == false;
        }
        if (action == Action.PressAndRelease || action == Action.OnlyRelease) {
            ret |= buttonPressed == false && lastButtonPressed == true;
        }

        // Store the current button state for use next time as the last state of the button
        if (button == 1) {
            lastButton1Pressed = buttonPressed;
        } else if (button == 2) {
            lastButton2Pressed = buttonPressed;
        }

        return ret;
    }

    // Updates the state value based upon the requested direction
    // Advance moves the state forward, as opposed to reverse which goes backwards
    private void updateState(boolean advance) {
        int change = advance ? delta : -delta; // Adjust change value based on direction

        currentState += change; // Advance state counter

        // Fix state count if it exceeded allowable range
        if (currentState < 0) {
            if (limit == LimitBehavior.Wrap) {
                currentState += maxState;
            } else {
                currentState = 0;
            }
        }
        if (currentState >= maxState) {
            if (limit == LimitBehavior.Wrap) {
                currentState = currentState % maxState;
            } else {
                currentState = maxState - 1;
            }
        }

        // Reverse direction if limit type is reverse and we are at either limit
        if (limit == LimitBehavior.Reverse) {
            if (currentState == 0 || currentState == (maxState - 1)) {
                delta = -delta;
            }
        }
    }
}
