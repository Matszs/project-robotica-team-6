

$(function() {

    var canvas = document.getElementById("canvas");
    var canvasWidth = canvas.width;
    var canvasHeight = canvas.height;
    var ctx = canvas.getContext("2d");
    var canvasData = ctx.getImageData(0, 0, canvasWidth, canvasHeight);

// That's how you define the value of a pixel //
    function drawPixel (x, y, r, g, b, a) {
        var index = (x + y * canvasWidth) * 4;

        canvasData.data[index + 0] = r;
        canvasData.data[index + 1] = g;
        canvasData.data[index + 2] = b;
        canvasData.data[index + 3] = a;
    }

// That's how you update the canvas, so that your //
// modification are taken in consideration //
    function updateCanvas() {
        ctx.putImageData(canvasData, 0, 0);
    }



    // Create a client instance
    var client = new Paho.MQTT.Client("ws.derfu.nl/ws/", 80, "clientId"+Math.floor((Math.random() * 10) + 1));

// set callback handlers
    client.onConnectionLost = onConnectionLost;
    client.onMessageArrived = onMessageArrived;

// connect the client
    client.connect({onSuccess:onConnect});


// called when the client connects
    function onConnect() {
        // Once a connection has been made, make a subscription and send a message.
        console.log("onConnect");
        client.subscribe("ros/vision/tracked_position");
        client.subscribe("ros/odom");

    }

// called when the client loses its connection
    function onConnectionLost(responseObject) {
        if (responseObject.errorCode !== 0) {
            console.log("onConnectionLost:"+responseObject.errorMessage);
        }
    }

    function publishColor(){
        var low = $("#low").val();
        var high = $("#high").val();

        console.log("low = "+ low +", high = "+ high);

        message = new Paho.MQTT.Message(low+";"+high);
        message.destinationName = "ros/test";
        client.send(message);
    }


// called when a message arrives
    function onMessageArrived(message) {
        //console.log("onMessageArrived:"+message.payloadString);
        var json = jQuery.parseJSON( message.payloadString );

        if(message.destinationName == 'ros/odom') {

            console.log(parseInt(json['pose']['pose']['position']['x'] * 10) + 100);
            console.log(parseInt(json['pose']['pose']['position']['y'] * 10) + 100);
            console.log('----');

            //drawPixel(json['pose']['pose']['position']['x'] + 100, json['pose']['pose']['position']['z'] + 100, 100, 0, 0, 0, 1);
            drawPixel(parseInt(json['pose']['pose']['position']['x'] * 10) + 100, parseInt(json['pose']['pose']['position']['y'] * 10) + 100, 255, 0, 0, 255);
            updateCanvas();

            console.log('kaas');

        }
    }








});


