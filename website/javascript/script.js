var client;
var ctx;

var addedX, addedY;
var zoomValue = 1;

var obstacleDetection;
var obstacleDetectionContext;



$(function() {

    obstacleDetection = document.getElementById("obstacle-detection");
    obstacleDetectionContext = obstacleDetection.getContext("2d");
    obstacleDetectionContext.translate(obstacleDetection.width / 2, obstacleDetection.height / 2); // center




    /*obstacleDetectionContext.clearRect(0, 0, obstacleDetection.width, obstacleDetection.height);

    for(var i = 0; i < 360; i++) {

        var value = 10 * 7;

        var radiansStart = i * (Math.PI / 180) - (Math.PI / 2);
        var radiansEnd = (i + 1) * (Math.PI / 180) - (Math.PI / 2);


        obstacleDetectionContext.beginPath();
        obstacleDetectionContext.moveTo(0, 0);
        obstacleDetectionContext.lineTo(value * Math.cos(Math.PI * i / 180.0), value * Math.sin(Math.PI * i / 180.0));
        obstacleDetectionContext.strokeStyle = 'rgb(0, 255, 0)';
        obstacleDetectionContext.lineWidth = 0.5;
        obstacleDetectionContext.stroke();

    }*/



});


