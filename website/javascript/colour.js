
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