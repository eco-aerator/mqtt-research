var mqtt = require('mqtt')
require('dotenv').config();

var options = {
    host: process.env.host,
    port: process.env.port,
    protocol: process.env.protocol,
    username: process.env.username,
    password: process.env.password
}

// initialize the MQTT client
var client = mqtt.connect(options);

// setup the callbacks
client.on('connect', function () {
    console.log('Connected');
});

client.on('error', function (error) {
    console.log(error);
});

client.on('message', function (topic, message) {
    // called each time a message is received
    console.log('Received message:', topic, message.toString());
});

// publish message 'Hello' to topic 'my/test/topic'
client.publish('my/test/topic', 'Hello');