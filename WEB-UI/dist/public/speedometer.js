/*
    Gauges for MARV Web Operator
    
    by Linus Johansson and Joakim Osterman, Spring 2022
*/


// Made with https://bernii.github.io/gauge.js/

/* APS gauge */
var target = document.getElementById('aps');    // Canvas element
var aps_gauge = new Gauge(target).setOptions({
            angle: 0.15,            // The span of the gauge arc
            lineWidth: 0.3,         // The line thickness
            radiusScale: 1,         // Relative radius
            pointer: {
                length: 0.59,           // Relative to gauge radius
                strokeWidth: 0.053,     // The thickness
                color: '#000000'        // Fill color
            },
            limitMax: false,            // If false, max value increases automatically if value > maxValue
            limitMin: false,            // If true, the min value of the gauge will be fixed
            colorStart: '#2DE000',      // Colors
            colorStop: '#2DE000',       // just experiment with them
            strokeColor: '#E0E0E0',     // to see which ones work best for you
            generateGradient: true,
            highDpiSupport: true,       // High resolution support  
});

aps_gauge.maxValue = 100;       // set max gauge value
aps_gauge.setMinValue(0);       // Prefer setter over gauge.minValue = 0
aps_gauge.animationSpeed = 10;  // set animation speed (32 is default value)
aps_gauge.set(0);               // set initial value

const aps_value_text = document.getElementById("aps-value");
aps_value_text.innerHTML = "0%";

/* RPS gauge */
var target = document.getElementById('rps');
var rps_gauge = new Gauge(target).setOptions({
            angle: 0.15,
            lineWidth: 0.3,
            radiusScale: 1,
            pointer: {
                length: 0.59,
                strokeWidth: 0.053,
                color: '#000000'
            },
            limitMax: false,
            limitMin: false,
            colorStart: '#E00000',
            colorStop: '#E00000',
            strokeColor: '#E0E0E0',
            generateGradient: true,
            highDpiSupport: true,
});

rps_gauge.maxValue = 100;
rps_gauge.setMinValue(0);
rps_gauge.animationSpeed = 10;
rps_gauge.set(0);

const rps_value_text = document.getElementById("rps-value");
rps_value_text.innerHTML = "0%";

/* Angle gauge */
var target = document.getElementById('angle');
var angle_gauge = new Gauge(target).setOptions({
            angle: 0.29,
            lineWidth: 0.3,
            radiusScale: 1,
            pointer: {
                length: 0.59,
                strokeWidth: 0.055,
                color: '#000000'
            },
            limitMax: false,
            limitMin: false,
            colorStart: '#E0E0E0',
            colorStop: '#E0E0E0',
            strokeColor: '#E0E0E0',
            generateGradient: true,
            highDpiSupport: true,

            renderTicks: {
                divisions: 2,
                divWidth: 4.2,
                divLength: 0.5,
                divColor: '#000000',
                subDivisions: 11,
                subLength: 0.5,
                subWidth: 0.6,
                subColor: '#1F1F1F'
            }
});
angle_gauge.maxValue = 22;
angle_gauge.setMinValue(-22);
angle_gauge.animationSpeed = 10;
angle_gauge.set(0);

const angle_value_text = document.getElementById("angle-value");
angle_value_text.innerHTML = "0°";


/* Display feedback values on gauges */
socket.on("feedback", (msg) => {
    const message = JSON.stringify(msg).split(",")
    const aps = parseFloat(message[0].split(" ")[1])
    const rps = parseFloat(message[1].split(" ")[2])
    const angle = parseFloat(message[2].split(" ")[2])

    aps_gauge.set(aps * 100); // set actual value
    rps_gauge.set(rps * 100); // set actual value
    angle_gauge.set(-angle); // set actual value

    aps_value_text.innerHTML = (aps * 100).toFixed(0) + "%";
    rps_value_text.innerHTML = (rps * 100).toFixed(0) + "%";
    angle_value_text.innerHTML = angle.toFixed(2) + "°";
});