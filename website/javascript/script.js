var client;
var ctx;

var addedX, addedY;
var zoomValue = 1;


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
    var canvasWidth = window.innerWidth;
    var canvasHeight = window.innerHeight;
    ctx = canvas.getContext("2d");
    var canvasData = ctx.getImageData(0, 0, canvasWidth, canvasHeight);
    var dots = [];
    var img = new Image();
    img.src = "/img/kobuki.png";

    ctx.canvas.width = canvasWidth;
    ctx.canvas.height = canvasHeight;

    ctx.translate(canvas.width/2,canvas.height/2);

    console.log(canvasWidth);
    console.log(canvasHeight);

    var draw = function() {
        var x = addedX;
        var y = addedY;

        ctx.clearRect(-canvasWidth, -canvasHeight, canvasWidth * 2, canvasHeight * 2);
        var lastX, lastY, angle;
        var headlen = 10;   // length of head in pixels

        for(var dotIndex in dots) {
            if(dotIndex == (dots.length - 1))
                continue;
            var dot = dots[dotIndex];

            ctx.beginPath();
            ctx.arc(dot['x'], dot['y'], 4, 0, 2 * Math.PI, false);
            ctx.fillStyle = '#ff0000';
            ctx.fill();

            if(typeof(lastX) !== 'undefined' && typeof(lastY) !== 'undefined') {

                angle = Math.atan2(dot['y'] - lastY, dot['x'] - lastX);

                ctx.strokeStyle = '#72d8ff';
                ctx.lineWidth = 3;

                ctx.beginPath();
                ctx.moveTo(lastX, lastY);
                ctx.lineTo(dot['x'], dot['y']);
                ctx.lineTo(dot['x'] - headlen * Math.cos(angle - Math.PI / 6), dot['y'] - headlen * Math.sin(angle - Math.PI / 6));
                ctx.stroke();

                ctx.beginPath();
                ctx.moveTo(dot['x'], dot['y']);
                ctx.lineTo(dot['x'] - headlen * Math.cos(angle + Math.PI / 6), dot['y'] - headlen * Math.sin(angle + Math.PI / 6));
                ctx.stroke();

            }

            lastX = dot['x'];
            lastY = dot['y'];
        }

        ctx.beginPath();
        ctx.drawImage(img, 0, 0, 307, 307, (x - 15), (y - 15), 30, 30);

        if(typeof(lastX) !== 'undefined' && typeof(lastY) !== 'undefined') {

            angle = Math.atan2(y - lastY, x - lastX);

            ctx.strokeStyle = '#72d8ff';
            ctx.lineWidth = 3;

            ctx.beginPath();
            ctx.moveTo(lastX, lastY);
            ctx.lineTo(x, y);
            ctx.lineTo(x - headlen * Math.cos(angle - Math.PI / 6), y - headlen * Math.sin(angle - Math.PI / 6));
            ctx.stroke();
            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(x - headlen * Math.cos(angle + Math.PI / 6), y - headlen * Math.sin(angle + Math.PI / 6));
            ctx.stroke();

            ctx.font = "20px Arial";
            ctx.fontStyle = '#ff363b';
            var degrees = parseInt(angle * (180 / Math.PI));
            if(degrees < 0)
                degrees += 360;
            degrees += 90;
            if(degrees > 360)
                degrees -= 360;

            ctx.fillText(degrees + " °", x + 20, y + 20);
            $('#text-degrees').text(degrees + " °");

        }


    };

    function drawPixel (x, y) {
        addedX = x;
        addedY = y;

        dots.push({'x' : x, 'y' : y});
        draw();
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
            //drawPixel(parseInt(canvas.width / 2 + json['pose']['pose']['position']['x']), canvas.height / 2 + parseInt(json['pose']['pose']['position']['y'] * 10));
            drawPixel(-parseInt(json['pose']['pose']['position']['x'] * 1), -parseInt(json['pose']['pose']['position']['y'] * 1));

        }
    }



    $('#zoom a[data-zoom]').click(function(e) {
        e.preventDefault();

        var value = parseFloat($(this).attr('data-zoom'));

        zoomValue = zoomValue + value;
        if(value === 0)
            zoomValue = 1;

        if(zoomValue < 0.4)
            zoomValue = 0.4;
        if(zoomValue > 1.6)
            zoomValue = 1.6;

        ctx.canvas.width = (window.innerWidth * zoomValue);
        ctx.canvas.height = (window.innerHeight * zoomValue);

        ctx.translate(ctx.canvas.width/2,ctx.canvas.height/2);

        draw();

    });

});


