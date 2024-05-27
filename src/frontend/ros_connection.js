const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', () => {
    console.log('Connected to websocket server.');
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', () => {
    console.log('Connection to websocket server closed.');
});

const cameraListener = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/image/compressed',
    messageType: 'sensor_msgs/CompressedImage'
});

const latencyListener = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/latency',
    messageType: 'std_msgs/Float64'
});

cameraListener.subscribe((message) => {
    const videoElement = document.getElementById('cameraStream');
    videoElement.src = 'data:image/jpeg;base64,' + message.data;
});

latencyListener.subscribe((message) => {
    document.getElementById('latency').innerText = `Latency: ${message.data.toFixed(3)} seconds`;
});

const linearSpeedTopic = new ROSLIB.Topic({
    ros: ros,
    name: 'linear_speed',
    messageType: 'std_msgs/Float32'
});

const angularSpeedTopic = new ROSLIB.Topic({
    ros: ros,
    name: 'angular_speed',
    messageType: 'std_msgs/Float32'
});

function publishLinearSpeed(speed) {
    const message = new ROSLIB.Message({
        data: speed
    });
    linearSpeedTopic.publish(message);
    console.log(`Published linear speed: ${speed}`);
}

function publishAngularSpeed(speed) {
    const message = new ROSLIB.Message({
        data: speed
    });
    angularSpeedTopic.publish(message);
}

document.getElementById('btnW').addEventListener('click', () => publishLinearSpeed(70));
document.getElementById('btnA').addEventListener('click', () => publishAngularSpeed(70));
document.getElementById('btnS').addEventListener('click', () => publishLinearSpeed(-70));
document.getElementById('btnD').addEventListener('click', () => publishAngularSpeed(-70));

document.addEventListener('keydown', (event) => {
    switch (event.key) {
        case 'w':
            publishLinearSpeed(70);
            break;
        case 'a':
            publishAngularSpeed(70);
            break;
        case 's':
            publishLinearSpeed(-70);
            break;
        case 'd':
            publishAngularSpeed(-70);
            break;
    }
});

document.addEventListener('keyup', (event) => {
    switch (event.key) {
        case 'w':
        case 's':
            publishLinearSpeed(0);
            break;
        case 'a':
        case 'd':
            publishAngularSpeed(0);
            break;
    }
});
