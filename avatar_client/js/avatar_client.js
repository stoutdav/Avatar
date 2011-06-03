// Globals
var FIELD_SEPARATOR = ",";
var COMMAND_SEPARATOR = ";";

// Messages from Avatar
var COMM_ERROR = 0;
var ACK = 1;
var ERR = 3;
var DEBUG_MESSAGE = 4;
var FRONT_COLLISION = 5;
var RAMP_SPEED = 6;
var MOTION_MULTIPLIER = 7;
var ROTATION_DISTANCE = 8;
var COLLISION_DISTANCE = 9;
var RESET_ACK = 10;

// Commands sent to Avatar
var IS_AVATAR_READY = 2;
var SEND_ALL_PARAMS = 11;  // Used when requesting all parameter values
var JOYSTICK = 12;
var SET_DEBUG = 13;
var SET_RAMP_SPEED = 14;
var SET_COLLISION_DISTANCE = 15;
var SET_ROTATION_DISTANCE = 16;
var SET_MOTION_MULTIPLIER = 17;
var RESET = 18;

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
        messageBuffer.push(String.fromCharCode(data[i]));
    }
    processMessages();
});

function processMessages() {
    var indexOfCommandSeparator = messageBuffer.indexOf(COMMAND_SEPARATOR);

    while (indexOfCommandSeparator != -1) {
        var message = getNextMessageFromBuffer();
        handleMessageFromServer(message);
        indexOfCommandSeparator = messageBuffer.indexOf(COMMAND_SEPARATOR);
    }
}

function getNextMessageFromBuffer() {
    var message = [];
    var length = messageBuffer.length;
    for (var i = 0; i < length; i++) {
        var nextChar = messageBuffer.shift();
        if (nextChar != COMMAND_SEPARATOR) {
            message.push(nextChar);
        } else {
            break;
        }
    }
    return message;
}

function handleMessageFromServer(message) {
    var logMessage;
    var messageStr = message.join("");
    var params = messageStr.split(FIELD_SEPARATOR);
    var messageId = parseInt(params.shift());

    switch (messageId) {
        case COMM_ERROR:
            displayMessage("COMM ERROR: " + params[0]);
            break;
        case ACK:
            displayMessage("ACK: " + params[0]);
            break;
        case ERR:
            displayMessage("ERROR: " + params[0]);
            break;
        case DEBUG_MESSAGE:
            $("#debugOutput").prepend("<br>");
            $("#debugOutput").prepend(params[0]);
            logMessage = params[0];
            break;
        case FRONT_COLLISION:
            displayMessage("Collision warning. Front sensor is " + params[0] + " cm from object.");
            break;
        case RAMP_SPEED:
            $("#rampSpeedSlider").slider("value", params[0]);
            $("#rampSpeed").val(params[0]);
            logMessage = "Set ramp speed to " + params[0];
            break;
        case MOTION_MULTIPLIER:
            $("#motionMultiplierSlider").slider("value", params[0]);
            $("#motionMultiplier").val(params[0]);
            logMessage = "Set motion multiplier to " + params[0];
            break;
        case ROTATION_DISTANCE:
            $("#rotationDistanceSlider").slider("value", params[0]);
            $("#rotationDistance").val(params[0]);
            logMessage = "Set rotation distance to " + params[0];
            break;
        case COLLISION_DISTANCE:
            $("#frontSensorSlider").slider("value", params[0]);
            $("#frontSensor").val(params[0]);
            logMessage = "Set front sensor distance to " + params[0];
            break;
        case RESET_ACK:
            break;

        default:
            logMessage = "Unknown Message: " + messageId;
    }
    console.log("Client: Received message from server", logMessage);
}

function displayMessage(logMessage) {
    $("#messages").prepend("<br>");
    $("#messages").prepend(logMessage);
}
function sendJoystickPosition(x, y) {
    // 80 comes from avatar.css
    var xScaled = Math.round((x / 80) * 10);
    var yScaled = Math.round((y / 80) * 10);
    sendMessageToServer(JOYSTICK, new Array(xScaled, yScaled));
}

function sendMessageToServer(command, fields) {
    console.log("Client Sending message to server command:", command, " params: ", fields.toString());
    var message = command;
    for (var i = 0; i < fields.length; i++) {
        message += FIELD_SEPARATOR;
        message += fields[i];
    }
    message += COMMAND_SEPARATOR;
    socket.send(message);
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
                    sendJoystickPosition(0, 0);
                }
            });

    $("#debug").buttonset();
    $("#debug").change(function() {
        var checkedValue = $("input[name=debug]:checked").val();
        $("#debugPopout").dialog("open");

        sendMessageToServer(SET_DEBUG, new Array(checkedValue));
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
                    sendMessageToServer(SET_DEBUG, new Array(DEBUG_OFF));
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
        sendMessageToServer(RESET, new Array());
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
                    sendMessageToServer(SET_RAMP_SPEED, [ui.value]);
                }
            });
    $("#rampSpeed").val($("#rampSpeedSlider").slider("value"));

    $("#motionMultiplierSlider").slider({
                range: "min",
                min: 1,
                max: 20,
                value: 3,
                slide: function(event, ui) {
                    $("#motionMultiplier").val(ui.value);
                },
                stop: function(event, ui) {
                    sendMessageToServer(SET_MOTION_MULTIPLIER, [ui.value]);
                }
            });
    $("#motionMultiplier").val($("#motionMultiplierSlider").slider("value"));

    $("#rotationDistanceSlider").slider({
                range: "min",
                min: 1,
                max: 20,
                value: 3,
                slide: function(event, ui) {
                    $("#rotationDistance").val(ui.value);
                },
                stop: function(event, ui) {
                    sendMessageToServer(SET_ROTATION_DISTANCE, [ui.value]);
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
                    sendMessageToServer(SET_COLLISION_DISTANCE, [ui.value]);
                }
            });
    $("#frontSensor").val($("#frontSensorSlider").slider("value"));

    // Once gui is setup get actual values from arduino
    sendMessageToServer(SEND_ALL_PARAMS, new Array());
});