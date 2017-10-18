var client;



function hex2rgb(hex) {
    if (hex[0]=="#") hex=hex.substr(1);
    if (hex.length==3) {
        var temp=hex; hex='';
        temp = /^([a-f0-9])([a-f0-9])([a-f0-9])$/i.exec(temp).slice(1);
        for (var i=0;i<3;i++) hex+=temp[i]+temp[i];
    }
    var triplets = /^([a-f0-9]{2})([a-f0-9]{2})([a-f0-9]{2})$/i.exec(hex).slice(1);
    return {
        red:   parseInt(triplets[0],16),
        green: parseInt(triplets[1],16),
        blue:  parseInt(triplets[2],16)
    }
}

function rgb2hsb(r, g, b)
{
    r /= 255; g /= 255; b /= 255; // Scale to unity.
    var minVal = Math.min(r, g, b),
        maxVal = Math.max(r, g, b),
        delta = maxVal - minVal,
        HSB = {hue:0, sat:0, bri:maxVal},
        del_R, del_G, del_B;

    if( delta !== 0 )
    {
        HSB.sat = delta / maxVal;
        del_R = (((maxVal - r) / 6) + (delta / 2)) / delta;
        del_G = (((maxVal - g) / 6) + (delta / 2)) / delta;
        del_B = (((maxVal - b) / 6) + (delta / 2)) / delta;

        if (r === maxVal) {HSB.hue = del_B - del_G;}
        else if (g === maxVal) {HSB.hue = (1 / 3) + del_R - del_B;}
        else if (b === maxVal) {HSB.hue = (2 / 3) + del_G - del_R;}

        if (HSB.hue < 0) {HSB.hue += 1;}
        if (HSB.hue > 1) {HSB.hue -= 1;}
    }

    HSB.hue *= 180;
    HSB.sat *= 100;
    HSB.bri *= 100;

    return HSB;
}


function hex2hsb(hex1){
    var rgb = hex2rgb(hex1);

    return rgb2hsb(rgb.red, rgb.green, rgb.blue);
}

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
    client = new Paho.MQTT.Client("ws.derfu.nl/ws/", 80, "clientId"+Math.floor((Math.random() * 10) + 1));

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

    $('#set-color').click(function() {

        var hsb_min = hex2hsb($("#low").val());
        var hsb_max = hex2hsb($("#high").val());

        message = new Paho.MQTT.Message(JSON.stringify({hue_low: parseInt(hsb_min['hue']), hue_high: parseInt(hsb_max['hue']), sat_low: parseInt(hsb_min['sat'] * 2.55), sat_high: parseInt(hsb_max['sat'] * 2.55), val_low: parseInt(hsb_min['bri'] * 2.55), val_high: parseInt(hsb_max['bri'] * 2.55)}));
        message.destinationName = "ros/vision/set_tracking_colours";
        client.send(message);
    });


// called when a message arrives
    function onMessageArrived(message) {
        //console.log("onMessageArrived:"+message.payloadString);
        var json = jQuery.parseJSON( message.payloadString );

        if(message.destinationName == 'ros/odom') {

            //drawPixel(json['pose']['pose']['position']['x'] + 100, json['pose']['pose']['position']['z'] + 100, 100, 0, 0, 0, 1);
            drawPixel(parseInt(json['pose']['pose']['position']['x'] * 10) + 100, parseInt(json['pose']['pose']['position']['y'] * 10) + 100, 255, 0, 0, 255);
            updateCanvas();

        }
    }





});


