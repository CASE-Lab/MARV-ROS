/*
    Gamepad input script for MARV Web Operator

    by Linus Johansson and Joakim Osterman, Spring 2022
*/

let enabled = false;

window.addEventListener("gamepadconnected", event => {
    console.log("Gamepad connected:")
    console.log(event.gamepad)
})

window.addEventListener("gamepaddisconnected", event => {
    console.log("Gamepad disconnected:")
    console.log(event.gamepad)
})

const enableGamepadButton = document.querySelector("#enable-gamepad");
enableGamepadButton.addEventListener("click", enableGamepad)

// Enable / Disable gamepad input from operator
function enableGamepad() {
    enabled = !enabled;

    if (enabled) {
        document.getElementById('enable-gamepad').innerHTML = "Disable Gamepad";
        console.log("Gamepad input enabled");
    } else {
        document.getElementById('enable-gamepad').innerHTML = "Enable Gamepad";
        console.log("Gamepad input disabled");
    }
}


function update() {
    const gamepads = navigator.getGamepads()
    if (gamepads[0] && enabled) {
        // Send state of lTrig, rTrig, lJoy, button0, button3 over socket
        socket.emit('gamepad', JSON.stringify({ lTrig: gamepads[0].buttons[6].value.toFixed(2),
                                                rTrig: gamepads[0].buttons[7].value.toFixed(2),
                                                lJoy: gamepads[0].axes[0].toFixed(2),
                                                button0: gamepads[0].buttons[0].pressed,
                                                button3: gamepads[0].buttons[3].pressed, }), null, 2);
    }
    window.requestAnimationFrame(update)
}

window.requestAnimationFrame(update)