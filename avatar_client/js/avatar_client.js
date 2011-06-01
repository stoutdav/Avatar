// Globals
var START_CHAR = "!";
var STOP_CHAR = "?";

//Constants
var COLLISION_WARNING = "W";
var DEBUG_MESSAGE = "#";

// Parameter constants. Must match constants in arduino avatar.pde file. Also used when setting a param in GUI
var RAMP_SPEED = 'A'; //A for acceleration
var MAXIMUM_SPEEP = 'M';
var FORWARD_DISTANCE = 'F';
var REVERSE_DISTANCE = 'B';
var ROTATION_DISTANCE = 'R';
var COLLISION_DISTANCE = 'C';
var RESET = 'T';

var SEND_ALL_PARAMS = 'Y';  // Used when requesting all parameter values

var JOYSTICK = 'j';
var POSITION = 'x';

var SET_DEBUG = 'D';
var DEBUG_OFF = '0';
var DEBUG_ON = '1';

// Receiving messages
var messageBuffer = [];
var socket = new io.Socket();
socket.connect();
socket.on('connect', function() {
    console.log("Client: Client connected to server");
});

socket.on('disconnect', function() {
    console.log("Client: Client disconnected from server");
});

socket.on('message', function(data) {
    // Need to build message because it comes back in chunks
    for (var i = 0; i < data.length; i++) {
        messageBuffer.push(data[i]);
    }
    processMessages();
});

function processMessages() {
    var indexOfStartChar = messageBuffer.indexOf(START_CHAR);
    var indexOfStopChar = messageBuffer.indexOf(STOP_CHAR);

    while (indexOfStartChar != -1 && indexOfStopChar != -1) {
        var message = getNextMessageFromBuffer();
        handleMessageFromServer(message);
        indexOfStartChar = messageBuffer.indexOf(START_CHAR);
        indexOfStopChar = messageBuffer.indexOf(STOP_CHAR);
    }
}

function getNextMessageFromBuffer() {
    var message = [];
    var length = messageBuffer.length;
    for (var i = 0; i < length; i++) {
        var nextChar = messageBuffer.shift();
        if (nextChar == STOP_CHAR) {
            break;
        }
        if (nextChar == START_CHAR) {
            continue;
        }
        message.push(nextChar);
    }
    return message;
}
function handleMessageFromServer(code) {
    var message;
    var messageType = code[0];
    var param = (code.splice(1, code.length - 1)).join("");
    switch (messageType) {
        case COLLISION_WARNING:
            message = "Collision warning. Front sensor is " + param + " cm from object.";
            $("#messages").prepend("<br>");
            $("#messages").prepend(message);
            break;
        case DEBUG_MESSAGE:
            $("#debugOutput").prepend("<br>");
            $("#debugOutput").prepend(param);
            message = param;
            break;
        case RAMP_SPEED:
            $("#rampSpeedSlider").slider("value", param);
            $("#rampSpeed").val(param);
            message = "Set ramp speed to " + param;
            break;
        case MAXIMUM_SPEEP:
            $("#maxSpeedSlider").slider("value", param);
            $("#maxSpeed").val(param);
            message = "Set maximum speed to " + param;
            break;
        case FORWARD_DISTANCE:
            $("#forwardDistanceSlider").slider("value", param);
            $("#forwardDistance").val(param);
            message = "Set forward distance to " + param;
            break;
        case REVERSE_DISTANCE:
            $("#reverseDistanceSlider").slider("value", param);
            $("#reverseDistance").val(param);
            message = "Set reverse distance to " + param;
            break;
        case ROTATION_DISTANCE:
            $("#rotationDistanceSlider").slider("value", param);
            $("#rotationDistance").val(param);
            message = "Set rotation distance to " + param;
            break;
        case COLLISION_DISTANCE:
            $("#frontSensorSlider").slider("value", param);
            $("#frontSensor").val(param);
            message = "Set front sensor distance to " + param;
            break;
        default:
            message = "Unknown Message"

    }
    console.log("Client: Received message from server", message);
}

function sendJoystickPosition(x, y) {
    // 80 comes from avatar.css
    var xScaled = Math.round((x/80) * 10);
    var yScaled = Math.round((y/80) * 10);
    sendMessageToServer(JOYSTICK + xScaled + POSITION + yScaled);
}

function sendMessageToServer(message) {
    socket.send(START_CHAR + message + STOP_CHAR);
}

$(function() {
    $("#joystick").draggable({
                revert:true,
                containment: "parent",
                create: function() {
                    $(this).data("startLeft", parseInt($(this).css("left")));
                    $(this).data("startTop", parseInt($(this).css("top")));
                },
                drag: function(event, ui) {
                    var x = ui.position.left - parseInt($(this).data("startLeft"));
                    var y = -(ui.position.top - parseInt($(this).data("startTop")));
                    sendJoystickPosition(x, y);
                },
                stop: function(event, ui) {
                    sendJoystickPosition(0,0);
                }
            });

    $("#debug").buttonset();
    $("#debug").change(function() {
        var checkedValue = $("input[name=debug]:checked").val();
        $("#debugPopout").dialog("open");

        sendMessageToServer(SET_DEBUG + checkedValue);
    });

    $("#debugPopout").dialog({
                title: "Debug Output",
                position: "left, bottom",
                width: 375,
                modal: false,
                autoOpen: false,
                close: function(event, ui) {
                    $("input[name=debug][value=0]").attr("checked", true);
                    $("#debug").buttonset("refresh");
                    sendMessageToServer(SET_DEBUG + DEBUG_OFF);
                }
            });

    $("#sliders").button();
    $("#sliders").click(function() {
        $("#slidersPopout").dialog("open");
    });

    $("#slidersPopout").dialog({
                title: "Motor & Sensor Controls",
                position: "top, right",
                resizable: false,
                draggable: false,
                modal: true,
                show: "slide",
                hide: "fold",
                autoOpen: false
            });

    $("#reset").button();
    $("#reset").click(function() {
        sendMessageToServer(RESET)
    });

// The default values for each slider should match what's in the arduino avatar.pde file
    $("#rampSpeedSlider").slider({
                range: "min",
                min: 1,
                max: 100,
                value: 15,
                slide: function(event, ui) {
                    $("#rampSpeed").val(ui.value);
                },
                stop: function(event, ui) {
                    sendMessageToServer(RAMP_SPEED + ui.value);
                }
            });
    $("#rampSpeed").val($("#rampSpeedSlider").slider("value"));

    $("#maxSpeedSlider").slider({
                range: "min",
                min: 1,
                max: 100,
                value: 36,
                slide: function(event, ui) {
                    $("#maxSpeed").val(ui.value);
                },
                stop: function(event, ui) {
                    sendMessageToServer(MAXIMUM_SPEEP + ui.value);
                }
            });
    $("#maxSpeed").val($("#maxSpeedSlider").slider("value"));

    $("#forwardDistanceSlider").slider({
                range: "min",
                min: 1,
                max: 100,
                value: 20,
                slide: function(event, ui) {
                    $("#forwardDistance").val(ui.value);
                },
                stop: function(event, ui) {
                    sendMessageToServer(FORWARD_DISTANCE + ui.value);
                }
            });
    $("#forwardDistance").val($("#forwardDistanceSlider").slider("value"));

    $("#reverseDistanceSlider").slider({
                range: "min",
                min: 1,
                max: 100,
                value: 10,
                slide: function(event, ui) {
                    $("#reverseDistance").val(ui.value);
                },
                stop: function(event, ui) {
                    sendMessageToServer(REVERSE_DISTANCE + ui.value);
                }
            });
    $("#reverseDistance").val($("#reverseDistanceSlider").slider("value"));

    $("#rotationDistanceSlider").slider({
                range: "min",
                min: 1,
                max: 50,
                value: 5,
                slide: function(event, ui) {
                    $("#rotationDistance").val(ui.value);
                },
                stop: function(event, ui) {
                    sendMessageToServer(ROTATION_DISTANCE + ui.value);
                }
            });
    $("#rotationDistance").val($("#rotationDistanceSlider").slider("value"));

    $("#frontSensorSlider").slider({
                range: "min",
                min: 5,
                max: 100,
                value: 5,
                slide: function(event, ui) {
                    $("#frontSensor").val(ui.value);
                },
                stop: function(event, ui) {
                    sendMessageToServer(COLLISION_DISTANCE + ui.value);
                }
            });
    $("#frontSensor").val($("#frontSensorSlider").slider("value"));

// Once gui is setup get actual values from arduino
    sendMessageToServer(SEND_ALL_PARAMS);
});