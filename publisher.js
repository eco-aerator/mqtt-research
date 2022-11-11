var mqtt = require('mqtt')
require('dotenv').config();

var options = {
    host: '9ded35a8cfb84084be0c9fdda79d39a6.s2.eu.hivemq.cloud',
    port: 8883,
    protocol: 'mqtts',
    username: 'crusteamqtt',
    password: 'crusteamqtt2022'
}

var waterTemp = 33.4;
var waterPh = 6.7;
var waterDo = 23.4;
var waterSalinity = 32.1;

// coba-coba kirim buffer
// var toBuffer = require('typedarray-to-buffer');
// var message = [];
// message[0] = waterTemp;
// message[1] = waterPh;
// message[2] = waterDo;
// message[3] = waterSalinity;
// var arr = new Uint8Array(message)
// arr = toBuffer(arr)


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
client.publish('/topic', 'Water Temprature: '+waterTemp);
client.publish('/topic', 'Water Do: '+waterDo);
client.publish('/topic', 'Water Salinity: '+waterSalinity);
client.publish('/topic', 'Water Ph: '+waterPh);