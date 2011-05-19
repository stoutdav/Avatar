console.log("starting");

var http = require("http"), io = require("socket.io"), serialport = require("serialport"), fs = require("fs");

var server = http.createServer(function (req, res) {
			fs.readFile("index.htm", function (err, data) {
					if (err)
						return send404(res);
					res.writeHead(200, {
							"Content-Type" : "text/html"
						})
					res.write(data, 'utf8');
					res.end();
				});
		});

send404 = function (res) {
	res.writeHead(404);
	res.write('404');
	res.end();
};
server.listen(80, "127.0.0.1");

var socket = io.listen(server);
var webClient;
socket.on('connection', function (client) {
		console.log("Server: Client connected to server");
		client.on('message', function (event) {
				console.log("Server: Received message from client", event);
				sp.write(event);
			});
		client.on('disconnect', function () {
				console.log("Server: Client has disconnected");
			});
		webClient = client;
	});

var sp = new serialport.SerialPort("/dev/ttyS3", {
			parser : serialport.parsers.raw,
			baudrate : 9600
		});

sp.on('data', function (data) {
		webClient.send("avatar says: " + data);
	});

 