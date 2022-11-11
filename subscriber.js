var mqtt = require('mqtt')
require('dotenv').config();

var options = {
    host: '9ded35a8cfb84084be0c9fdda79d39a6.s2.eu.hivemq.cloud',
    port: 8883,
    protocol: 'mqtts',
    username: 'crusteamqtt',
    password: 'crusteamqtt2022'
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

// subscribe to topic 'my/test/topic'
client.subscribe('/topic');