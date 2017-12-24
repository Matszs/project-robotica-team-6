var gridBlocks = [];
var getCoordinatesFromMousePosition;

var drawGrid;
var drawLocation;
var gridToCenter;
var centerToGrid;
var locationX, locationY;
var updateCanvasSize;



$(function() {

    updateCanvasSize = function() {
        var height = (window.innerHeight * zoomValue);
        var width = (window.innerWidth * zoomValue);

        ctx.canvas.width = width;
        ctx.canvas.height = height;

        if(typeof(drawGrid) === 'function')
            drawGrid();

    };

    var canvas = document.getElementById("canvas");
    var canvasWidth = window.innerWidth;
    var canvasHeight = window.innerHeight;
    ctx = canvas.getContext("2d");
    var canvasData = ctx.getImageData(0, 0, canvasWidth, canvasHeight);
    var dots = [];
    var img = new Image();
    img.src = "/img/kobuki.png";
    var gridSize = 10;

    updateCanvasSize();

    gridToCenter = function(x, y) {

    };

    // example x = 0, y = 0
    centerToGrid = function(x, y) {
        var centerPoint = getCoordinatesFromMousePosition((ctx.canvas.width / 2), (ctx.canvas.height / 2));

        return {
            'x': centerPoint['x'] + x,
            'y': centerPoint['y'] + y
        };
    };

    var draw = function() {

    };

    drawGrid = function() {

        var canvasWidth = ctx.canvas.width;
        var canvasHeight = ctx.canvas.height;

        ctx.clearRect(0, 0, canvasWidth, canvasHeight);

        ctx.beginPath();

        for(var i = 0; i < (canvasHeight / gridSize); i++) {
            for(var j = 0; j < (canvasWidth / gridSize); j++) {
                ctx.rect(j * gridSize, i * gridSize, gridSize, gridSize);
            }
        }

        ctx.lineWidth = 0.5;
        ctx.strokeStyle = "#c8c5bb";
        ctx.stroke();



        for(var gridBlockIndex in gridBlocks) {
            var gridBlock = gridBlocks[gridBlockIndex];

            ctx.beginPath();
            ctx.rect(gridBlock.x * gridSize, gridBlock.y * gridSize, gridSize, gridSize);

            ctx.fillStyle = (gridBlock['type'] === 3 ? '#6141d0' : (gridBlock['type'] === 1 ? "#50D050" : "#f61f1f"));
            ctx.fill();
            ctx.strokeStyle = (gridBlock['type'] === 3 ? '#6141d0' : (gridBlock['type'] === 1 ? "#50D050" : "#f61f1f"));
            ctx.stroke();
        }

        drawLocation();
    };

    drawGridLast = function() {

        var gridBlock = gridBlocks[gridBlocks.length - 1];

        ctx.beginPath();
        ctx.rect(gridBlock.x * gridSize, gridBlock.y * gridSize, gridSize, gridSize);

        ctx.fillStyle = (gridBlock['type'] === 3 ? '#6141d0' : (gridBlock['type'] === 1 ? "#50D050" : "#f61f1f"));
        ctx.fill();
        ctx.strokeStyle = (gridBlock['type'] === 3 ? '#6141d0' : (gridBlock['type'] === 1 ? "#50D050" : "#f61f1f"));
        ctx.stroke();

        drawLocation();
    };

    drawLocation = function(x, y) {
        if(typeof(x) !== 'undefined')
            locationX = x;
        if(typeof(y) !== 'undefined')
            locationY = y;

        ctx.beginPath();
        ctx.rect(locationX * gridSize, locationY * gridSize, gridSize, gridSize);

        ctx.fillStyle = "#000000";
        ctx.fill();
        ctx.strokeStyle = "#000000";
        ctx.stroke();
    };

    drawGrid();



    var drawPixel = function(x, y) {
        addedX = x;
        addedY = y;

        dots.push({'x' : x, 'y' : y});
        draw();
    };


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

        updateCanvasSize();

    });

    $(window).resize(function() {
        updateCanvasSize();
    });

    getCoordinatesFromMousePosition = function(mouseX, mouseY) {
        return {
            'x': Math.floor(mouseX / (gridSize)),
            'y': Math.floor(mouseY / (gridSize))
        };
    };


    $('canvas').click(function(e) {

        var xPosition = e.clientX;
        var yPosition = e.clientY;

        var selectedGrid = getCoordinatesFromMousePosition(xPosition, yPosition);

        /*gridBlocks.push({
            'x': selectedGrid.x,
            'y': selectedGrid.y,
            'type': 0
        });

        drawGrid();*/
    });

});