// Global variables
var twist;
var cmdVel;
var float64;
var petAcc;
var publishNow = true;
var kitCatIP;
var joy;
var teleop;
var accSlider;
var ros;
var kLinear = 0.03;
var kAngular = 0.015;


// PUBLISHER that publishes the ROS Twist message with throttle and steering
function moveActionPublisher(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }

    cmdVel.publish(twist);
}

// PUBLISHER that publishes the ROS Float64 message with the accessory rotation
function accessoryMovePublisher(accessory) {
    if(float64 !== undefined) {
        float64.data = accessory;
    } else {
        float64.data = 0;
    }

    petAcc.publish(float64);
}


// ADVERTISER that initializes the publication to "/cmd_vel" topic
function initVelocityPublisher() {
    // Initial values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });

    // Init topic JS object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    // Register publisher within ROS system
    cmdVel.advertise();
}

// ADVERTISER that initializes the publication to "/pet_accessory" topic
function initAccessoryPublisher() {
    // Initial values.
    float64 = new ROSLIB.Message({
        data: 0
    });

    // Init topic JS object
    petAcc = new ROSLIB.Topic({
        ros: ros,
        name: '/pet_accessory',
        messageType: 'std_msgs/Float64'
    });

    // Register publisher within ROS system
    petAcc.advertise();
}


// Creates the joystick controller JS object from the nipplejs library
function initJoystick() {
    // Check if joystick controller was already created
    if (joy == null) {
        joystickContainer = document.getElementById('joystick');
        // Create joystick with configuration from https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 50 + '%', top: 105 + 'px' },
            mode: 'static',
            size: 200,
            color: '#f0ad4e',
            restJoystick: true,
            restOpacity: 0.7,
            threshold: 0.1,
            multitouch: false,
            maxNumberOfNipples: 1
        };
        joy = nipplejs.create(options);

        // Add a JS event listener for a joystick move
        joy.on('move', function (evt, nipple) {
            // Joystick angle is in screen coordinates (counter-clockwise with origin at the right of the horizontally axis) and has to be rotated -90º in order to obtain the rotation along the car longitudinal axis:
            var angRotated = nipple.angle.degree - 90;
            if (angRotated > 180) { // Fourth quadrant 
                angRotated = -(450 - nipple.angle.degree);
            }

            // Convert to radians and scale linear and angular commands by also applying adjustable the constants kLinear and kAngular
            var lin = Math.cos(angRotated * (Math.PI/180)) * nipple.distance * kLinear;
            var ang = Math.sin(angRotated * (Math.PI/180)) * nipple.distance * kAngular;

            // nipplejs triggers events when joystick moves each pixel, a 50 ms delay between ROS publications is implemented 
            if (publishNow) {
                publishNow = false;
                if (ang > 1) { ang = 1; }
                else if (ang < -1) { ang = -1; }
                moveActionPublisher(lin, ang);
                setTimeout(function () {
                    publishNow = true;
                }, 50);
            }
        });

        // Add a JS event listener to stop the car when the joystick is release
        joy.on('end', function () {
            moveActionPublisher(0, 0);
        });
    }
}

// Creates the alternative keyboard controller JS object from the robotwebtools library
function initTeleopKeyboard() {
    // Use w, s, a, d keys to move around (^,v,<,>)

    // Check if keyboard controller was already created
    if (teleop == null) {
        // Initialize the keyboard controller
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel'
        });
    }

    // Add a JS event listener for changes in the speed slider in case the keys are being used
    robotSpeedFromKeys = document.getElementById("robot-speed");
    robotSpeedFromKeys.oninput = function () {
        teleop.scale = robotSpeedFromKeys.value / 100
    }
}

// Creates the slider accessory controller JS object
function initAccessorySlider() {
    // Add a JS event listener for changes in the pet accessory slider
    accSlider = document.getElementById("accessory");
    accSlider.oninput = function () {
        accessoryMovePublisher(accSlider.value / 100);
    }
}


// MAIN
window.onload = function () {
    // Determine Kit-Cat's address automatically (or set it manually)
    kitCatIP = location.hostname;

    // Initialize the handler for for the rosbridge_websocket
    ros = new ROSLIB.Ros({
        url: "ws://" + kitCatIP + ":9090" // Default port for a ROS bridge web socket
    });

    initVelocityPublisher();
    initAccessoryPublisher();

    // Initialize handlers to start driving
    start = document.getElementById('start');
    video = document.getElementById('video');
    loadControllers = false;

    // Fill it with the video stream link generated by the node web_video_server, which is going to be listening for http requests in port 8887
    video.src = "http://" + kitCatIP + ":8887/stream?topic=/raspicam_node/image&image_transport=compressed&type=mjpeg";
    console.log("Camera stream URL generated");

    // Enable the joystick and keyboard controllers when the video is correclty loaded and the user has read the controller instructions
    start.onclick = function () {
        console.log("Closed instructions");
        setTimeout(() => console.log("Starting controllers"), 1000);
        setTimeout(() => initJoystick(), 1200);
        setTimeout(() => initTeleopKeyboard(), 1200);
        setTimeout(() => initAccessorySlider(), 1200);
    }

}