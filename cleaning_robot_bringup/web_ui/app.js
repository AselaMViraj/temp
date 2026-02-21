// Rover Web Commander - Logical Core

// 1. Connection Logic
const ros = new ROSLIB.Ros({
    url: `ws://${window.location.hostname}:9090`
});

const statusEl = document.getElementById('status');
ros.on('connection', () => {
    statusEl.innerText = 'Connected';
    statusEl.className = 'connected';
});

ros.on('error', (error) => {
    statusEl.innerText = 'Error Connecting';
    statusEl.className = 'disconnected';
    console.error(error);
});

ros.on('close', () => {
    statusEl.innerText = 'Disconnected';
    statusEl.className = 'disconnected';
});

// 2. Map Rendering
const mapDiv = document.getElementById('map');
const viewer = new ROS2D.Viewer({
    divID: 'map',
    width: mapDiv.clientWidth,
    height: mapDiv.clientHeight
});

// Create Global Occupancy Grid client
const gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    continuous: true
});

// Workaround for Transient Local Map (Static Map)
// Standard roslib/ros2djs subscription is Volatile, so it misses the latched map.
const mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/map',
    messageType: 'nav_msgs/OccupancyGrid',
    compression: 'png',
    qos: { reliability: 'reliable', durability: 'transient_local' }
});

mapTopic.subscribe((message) => {
    console.log("Received Static Map!");
    gridClient.processMessage(message);
    // Unsubscribe after getting the static map to save bandwidth
    mapTopic.unsubscribe();
});

// Add Local Costmap Layer (transparent)
const localCostmapClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    topic: '/local_costmap/costmap',
    continuous: true,
    opacity: 0.5
});

// Add Global Costmap Layer (transparent)
const globalCostmapClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    topic: '/global_costmap/costmap',
    continuous: true,
    opacity: 0.3
});

// Scale map to fit on initial load
let initialScaleDone = false;
gridClient.on('change', () => {
    if (!initialScaleDone && gridClient.currentGrid) {
        const width = gridClient.currentGrid.width;
        const height = gridClient.currentGrid.height;

        // Calculate proportional scale to fit viewer
        const scale = Math.min(
            viewer.width / width,
            viewer.height / height
        ) * 0.9; // 90% of available space

        viewer.scene.scaleX = scale;
        viewer.scene.scaleY = scale;

        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
        initialScaleDone = true;
    }
});

// 3. Interaction Logic
const zoomSpeed = 1.1;
mapDiv.addEventListener('wheel', (event) => {
    event.preventDefault();
    const zoom = event.deltaY > 0 ? 1 / zoomSpeed : zoomSpeed;
    viewer.scene.scaleX *= zoom;
    viewer.scene.scaleY *= zoom;
});

let isDragging = false;
let startPos = { x: 0, y: 0 };

viewer.scene.on('stagemousedown', (event) => {
    if (event.nativeEvent.button === 0) { // Left click for marking
        const pos = viewer.scene.globalToRos(event.stageX, event.stageY);
        // Smart Goal Snapping: Find nearest safe point if clicked on obstacle
        const safePos = findNearestSafe(pos.x, pos.y);
        tempPose = safePos;
        waypointForm.classList.remove('hidden');
        document.getElementById('waypoint-name').focus();
    } else { // Other clicks for panning
        isDragging = true;
        startPos = { x: event.stageX, y: event.stageY };
    }
});

viewer.scene.on('stagemousemove', (event) => {
    if (isDragging) {
        const dx = event.stageX - startPos.x;
        const dy = event.stageY - startPos.y;
        viewer.scene.x += dx;
        viewer.scene.y += dy;
        startPos = { x: event.stageX, y: event.stageY };
    }
});

viewer.scene.on('stagemouseup', () => {
    isDragging = false;
});

// Disable context menu to allow right-click panning
mapDiv.addEventListener('contextmenu', (e) => e.preventDefault());

// 4. ROS Communication
const addWaypointTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/add_waypoint',
    messageType: 'std_msgs/String'
});

const waypointsListTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/waypoints_list',
    messageType: 'std_msgs/String'
});

const navToWaypointTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/nav_to_waypoint',
    messageType: 'std_msgs/String'
});

const resetWaypointsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/reset_waypoints',
    messageType: 'std_msgs/String'
});

document.getElementById('reset-waypoints').addEventListener('click', () => {
    if (confirm("Are you sure you want to delete ALL waypoints? This cannot be undone.")) {
        const msg = new ROSLIB.Message({ data: "reset" });
        resetWaypointsTopic.publish(msg);
        console.log("Sent reset command");
    }
});

// Robot Marker (Detailed Top-Down View)
const robotMarker = new createjs.Container();

// 1. Chassis (Orange Box: 0.335m x 0.265m) - Inset slightly for visual clearance
const chassis = new createjs.Shape();
chassis.graphics.beginFill("#FF8C00").drawRect(-0.1675, -0.1325, 0.335, 0.265);
robotMarker.addChild(chassis);

// 2. Wheels (Blue Cylinders: ~0.06m x 0.026m)
const leftWheel = new createjs.Shape();
leftWheel.graphics.beginFill("#3333FF").drawRect(-0.06, 0.14, 0.12, 0.05); // Approximated visual size
robotMarker.addChild(leftWheel);

const rightWheel = new createjs.Shape();
rightWheel.graphics.beginFill("#3333FF").drawRect(-0.06, -0.19, 0.12, 0.05);
robotMarker.addChild(rightWheel);

// 3. Direction Indicator (Red Arrow at front)
const arrow = new createjs.Shape();
arrow.graphics.beginFill("red").moveTo(0.1, 0).lineTo(0, 0.05).lineTo(0, -0.05).closePath();
robotMarker.addChild(arrow);

robotMarker.visible = false;
viewer.scene.addChild(robotMarker);

const poseTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/amcl_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
});

// Smoothing variables
let targetX = 0;
let targetY = 0;
let targetRotation = 0;
const LERP_FACTOR = 0.02; // 2% movement per frame (much smoother)

poseTopic.subscribe((message) => {
    // Quaternion to Euler (Yaw)
    var q = message.pose.pose.orientation;
    var siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    var cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    var yaw = Math.atan2(siny_cosp, cosy_cosp);
    var deg = yaw * 180 / Math.PI;

    // Update targets instead of direct assignment
    targetX = message.pose.pose.position.x;
    targetY = -message.pose.pose.position.y; // Inverted Y for Canvas
    targetRotation = -deg; // Inverted rotation for Canvas

    // Make visible on first update
    if (!robotMarker.visible) {
        robotMarker.visible = true;
        robotMarker.x = targetX;
        robotMarker.y = targetY;
        robotMarker.rotation = targetRotation;
    }
});

// Animation Loop for Smoothing
createjs.Ticker.timingMode = createjs.Ticker.RAF;
createjs.Ticker.addEventListener("tick", () => {
    if (robotMarker.visible) {
        // Position LERP
        robotMarker.x += (targetX - robotMarker.x) * LERP_FACTOR;
        robotMarker.y += (targetY - robotMarker.y) * LERP_FACTOR;

        // Rotation LERP (handle wraparound)
        let diff = targetRotation - robotMarker.rotation;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        robotMarker.rotation += diff * LERP_FACTOR;
    }
    viewer.scene.update();
});

// 5. Waypoint Management
let tempPose = null;
const waypointForm = document.getElementById('waypoint-form');

document.getElementById('save-waypoint').addEventListener('click', () => {
    const name = document.getElementById('waypoint-name').value;
    if (name && tempPose) {
        const msg = new ROSLIB.Message({
            data: `${name},${tempPose.x},${tempPose.y},0.0,1.0`
        });
        addWaypointTopic.publish(msg);
        waypointForm.classList.add('hidden');
        document.getElementById('waypoint-name').value = '';
    }
});

document.getElementById('cancel-waypoint').addEventListener('click', () => {
    waypointForm.classList.add('hidden');
});

waypointsListTopic.subscribe((message) => {
    try {
        const waypoints = JSON.parse(message.data);
        updateWaypointsList(waypoints);
    } catch (e) { console.error("Error parsing waypoints:", e); }
});

function updateWaypointsList(waypoints) {
    const listEl = document.getElementById('waypoints-list');
    listEl.innerHTML = '';

    Object.keys(waypoints).forEach(name => {
        const wp = waypoints[name];
        const item = document.createElement('li');
        item.className = 'waypoint-item';
        item.innerHTML = `
            <span>${name}</span>
            <div class="waypoint-controls">
                <button class="btn-go" onclick="goToWaypoint('${name}')">Go</button>
            </div>
        `;
        listEl.appendChild(item);
    });
}

function goToWaypoint(name) {
    const msg = new ROSLIB.Message({
        data: name
    });
    navToWaypointTopic.publish(msg);
    console.log(`Requested navigation to ${name}`);
}

window.goToWaypoint = goToWaypoint;

window.addEventListener('resize', () => {
    viewer.width = mapDiv.clientWidth;
    viewer.height = mapDiv.clientHeight;
});

// 6. Safe Goal Finding Utility
function findNearestSafe(startX, startY) {
    if (!gridClient.currentGrid || !gridClient.currentGrid.message) return { x: startX, y: startY };

    const msg = gridClient.currentGrid.message;
    const res = msg.info.resolution;
    const width = msg.info.width;
    const height = msg.info.height;
    const ox = msg.info.origin.position.x;
    const oy = msg.info.origin.position.y;

    // Grid coordinates conversion
    const toGrid = (wx, wy) => ({
        x: Math.floor((wx - ox) / res),
        y: Math.floor((wy - oy) / res)
    });

    const toWorld = (gx, gy) => ({
        x: (gx * res) + ox + (res / 2),
        y: (gy * res) + oy + (res / 2)
    });

    const start = toGrid(startX, startY);
    const data = msg.data;

    // Check if start is valid and safe immediately
    const getIdx = (x, y) => y * width + x;
    const isValid = (x, y) => x >= 0 && x < width && y >= 0 && y < height;

    // Threshold: < 60 is safe (allowing for some cost but not lethal/inflated wall)
    // 0 is free, 100 is lethal.
    const isSafe = (x, y) => {
        if (!isValid(x, y)) return false;
        const val = data[getIdx(x, y)];
        return val !== -1 && val < 60;
    };

    if (isSafe(start.x, start.y)) return { x: startX, y: startY };

    console.log("Clicked on obstacle, searching for nearest safe point...");

    // BFS Search for nearest safe cell
    const key = (x, y) => `${x},${y}`;
    const visited = new Set([key(start.x, start.y)]);
    const queue = [start];

    let iterations = 0;
    const MAX_SEARCH = 2000; // Limit search preventing freeze

    while (queue.length > 0 && iterations < MAX_SEARCH) {
        const u = queue.shift();
        iterations++;

        if (isSafe(u.x, u.y)) {
            const found = toWorld(u.x, u.y);
            console.log(`Snapped to safe: ${found.x.toFixed(2)}, ${found.y.toFixed(2)}`);
            return found;
        }

        const neighbors = [
            { x: u.x + 1, y: u.y }, { x: u.x - 1, y: u.y },
            { x: u.x, y: u.y + 1 }, { x: u.x, y: u.y - 1 }
        ];

        for (const n of neighbors) {
            if (isValid(n.x, n.y) && !visited.has(key(n.x, n.y))) {
                visited.add(key(n.x, n.y));
                queue.push(n);
            }
        }
    }

    return { x: startX, y: startY }; // Fallback if nothing found
}
