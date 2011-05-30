console.log("starting");

var http = require("http"), io = require("socket.io"), serialport = require("serialport"), fs = require("fs"),
        path = require("path"), url = require("url"), opts = require("opts");

var options = [
    { short: "p"
        , long: "httpPort"
        , description: "Port for http server"
        , value: true
        , callback: function(value) {
        httpPort = value;
        console.log("Overriding default http port with", httpPort)
    }
    },
    { short: "s"
        , long: "serialPort"
        , description: "Serial port Arduino is connected"
        , value: true
        , callback: function(value) {
        serialPort = value;
        console.log("Overriding default serial port with", serialPort)
    }
    }
];

opts.parse(options, true);
var httpPort = opts.get("httpPort") || 8080;
var serialPort = opts.get("serialPort") || "/dev/ttyS3";

var server = http.createServer(function (request, response) {
    var filePath = "../avatar_client" + request.url;
    console.log("Request for file", filePath);

    if (filePath == "../avatar_client/")
        filePath = "../avatar_client/index.html";

    var fileExtension = path.extname(filePath);
    var contentType = "text/html";
    switch (fileExtension) {
        case ".js":
            contentType = "text/javascript";
            break;
        case ".css":
            contentType = "text/css";
            break;
        case ".ico":
            contentType = "image/x-icon";
            break;
        case ".png":
            contentType = "image/png";
            break;
    }
    path.exists(filePath, function (exists) {
        if (exists) {
            fs.readFile(filePath, function (error, content) {
                if (error) {
                    send500(response)
                } else {
                    console.log("Serving", filePath);
                    response.writeHead(200, {"Content-Type" : contentType});
                    response.write(content, 'utf8');
                    response.end();
                }
            });
        } else {
            send404(response);
        }
    });
});

send404 = function (response) {
    response.writeHead(404);
    response.write('404');
    response.end();
};
send500 = function (response) {
    response.writeHead(500);
    response.write('500');
    response.end();
};

server.listen(httpPort);
var socket = io.listen(server);
var webClient;

socket.on('connection', function (client) {
    console.log("Client connected to server");
    client.on('message', function (event) {
        console.log("Received message from client", event);
        sp.write(event);
    });
    client.on('disconnect', function () {
        console.log("Client has disconnected");
    });
    webClient = client;
});
var sp = new serialport.SerialPort(serialPort, {
            parser : serialport.parsers.raw,
            baudrate : 9600
        });

sp.on('data', function (data) {
    // Wait for a connection before trying to send responses
    if (webClient) {
        var message = new String(data);
        console.log("Sending a message to client: " + message);
        webClient.send(message);
    }
});
