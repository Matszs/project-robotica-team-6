$(function() {


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
        client.subscribe("ros/grid_field");
        client.subscribe("ros/location");

        $('#status').addClass('online').removeClass('offline');
    }

    // called when the client loses its connection
    function onConnectionLost(responseObject) {
        if (responseObject.errorCode !== 0) {
            console.log("onConnectionLost:"+responseObject.errorMessage);
        }

        $('#status').addClass('offline').removeClass('online');
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

        console.log(json);

        if(message.destinationName == 'ros/grid_field') {

            var selectedGrid = centerToGrid(parseInt(json['x']), parseInt(json['y']));

            gridBlocks.push({
                'x': selectedGrid.x,
                'y': selectedGrid.y,
                'type': parseInt(json['type'])
            });

            drawGrid();

        }

        if(message.destinationName == 'ros/location') {
            var selectedGrid = centerToGrid(parseInt(json['x']), parseInt(json['y']));

            drawLocation(selectedGrid.x, selectedGrid.y);
            drawGrid();
        }
    }


});