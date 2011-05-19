console.log("starting");

var http = require("http"), io = require("socket.io"), serialport = require("serialport"), fs = require("fs"), path = require("path");

var server = http.createServer(function (request, response) {
	var filePath = "." + request.url;
	console.log("Request for file", filePath);
			
	if (filePath == "./")
		filePath = "./index.htm"
					
	var fileExtension = path.extname(filePath);
	var contentType = "text/html";
	switch (fileExtension) {
		case ".js":
			contentType = "text/javascript";
			break;
		case ".css":
			contentType = "text/css";
			break;
	}
	path.exists(filePath, function (exists) {
		if (exists) {
			fs.readFile(filePath, function (error, content) {
				if (error) {
					send500(response)
				} else {
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
		responsewrite('500');
		response.end();
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
	
	 