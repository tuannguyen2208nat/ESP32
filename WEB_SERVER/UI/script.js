var gateway = `ws://${window.location.hostname}:8080/ws`;
var websocket;

window.addEventListener('load', onLoad);

function onLoad(event) {
    initWebSocket();
}

function toggleButton(index) {
    websocket.send("toggle" + index);
}

function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}


function onMessage(event) {
    var data;
    console.log(event.data);
    data = JSON.parse(event.data);
    if (data.relay_id) {
        var index = data.relay_id;
        var state = data.state;
        document.getElementById("state" + index).innerHTML = state;
    }
    if (data.getState1 && data.getState2 && data.getState3 && data.getState4) {
        document.getElementById("state1").innerHTML = data.getState1;
        document.getElementById("state2").innerHTML = data.getState2;
        document.getElementById("state3").innerHTML = data.getState3;
        document.getElementById("state4").innerHTML = data.getState4;
    }
    if (data.temperature && data.humidity && data.flame) {
        document.getElementById('temperature').innerHTML = data.temperature.toFixed(2);
        document.getElementById('humidity').innerHTML = data.humidity.toFixed(2);
        document.getElementById('flame').innerHTML = data.flame;
    }
}
