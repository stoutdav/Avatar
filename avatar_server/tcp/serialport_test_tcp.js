console.log("starting");

var serverConnection;
var net = require("net");
var server = net.createServer(function (connection) {
			connection.write('welcome to avatar\r\n');
			connection.on('data', function (data) {
					connection.write('sending command to avatar: ' + data + '\r');
					sp.write(data);
				});
			
			serverConnection = connection;
		});

server.listen(1337, "127.0.0.1");

var serialport = require("serialport");
var sp = new serialport.SerialPort("/dev/ttyS3", {
			parser : serialport.parsers.raw,
			baudrate : 9600
		});

sp.on('data', function (data) {
		serverConnection.write('avatar says: ' + data + '\r\n');
	});

 