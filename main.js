var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        //dragging data
        dragging: false,
        x: 'no',
        y: 'no',

        rosbridge_address: '',
        port: '9090',

        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
        },

        // joystick valules
        joystick: {
            vertical: 0,
            horizontal: 0,
        },
        //publisher
        pubInterval: null,
        position: { x: 0, y: 0, z: 0, },
        //generated map
        mapViewer: null,
        mapGridClient: null,
        interval: null,

        //3D model
        viewer3d: null,
        tfClient: null,
        urdfClient: null,

        //action
        goal: null,
        action: {
            goal: { position: { x: 0, y: 0, z: 0 } },
            feedback: { position: 0, state: 'idle' },
            result: { success: false },
            status: { status: 0, text: '' },

        }

    },
    // helper methods to connect to ROS
    methods: {
        connect: function () {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
                this.pubInterval = setInterval(this.sendCommand, 100)
                this.setCamera()
                this.setMap()
                this.set3DViewer()

                //odom sub
                let topic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/odom',
                    messageType: 'nav_msgs/Odometry'
                })
                topic.subscribe((message) => {
                    this.position = message.pose.pose.position
                    //console.log(message)
                })

            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                clearInterval(this.sendCommand)
                this.unsetMap()
                this.unsetCamera()
                this.unset3DViewer()
            })
        },
        disconnect: function () {
            this.ros.close()
        },

        sendCommand: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.joystick.horizontal, },
            })
            topic.publish(message)
        },

        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
        setCamera: function () {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: 280,
                height: 240,
                topic: '/camera/image_raw',
                ssl: true,
            })
        },

        setMap: function () {

            this.mapViewer = new ROS2D.Viewer({
                divID: 'map',
                width: 600,
                height: 300
            })
            // Setup the map client.
            this.mapGridClient = new ROS2D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.mapViewer.scene,
                continuous: true,
            })
            // Scale the canvas to fit to the map
            this.mapGridClient.on('change', () => {

                // Scale the map dimensions by a factor of 2 (adjust the factor as needed)
                const scaleFactor = 1;
                const newWidth = this.mapGridClient.currentGrid.width * scaleFactor;
                const newHeight = this.mapGridClient.currentGrid.height * scaleFactor;

                // Scale the map viewer to the new dimensions
                this.mapViewer.scaleToDimensions(newWidth, newHeight);

                // Shift the map viewer based on the grid's position
                this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x, this.mapGridClient.currentGrid.pose.position.y);

                // Logging the scaled dimensions (for debugging)
                //console.log(newWidth);
                //console.log(newHeight);

            })

        },

        set3DViewer: function () {
            this.viewer3d = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 280,
                height: 240,
                antialias: true,
                fixedFrame: 'odom'
            })
            // Add a grid.
            this.viewer3d.addObject(new ROS3D.Grid({
                color: '#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0
            })

            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: 'robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer3d.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })

        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
        },
        unsetMap() {
            document.getElementById('map').innerHTML = ''

        },
        unsetCamera() {

            document.getElementById('divCamera').innerHTML = ''
        },
        sendGoal: function () {

            let actionClient = new ROSLIB.ActionClient({
                ros: this.ros,
                serverName: '/tortoisebot_as',
                actionName: 'course_web_dev_ros/WaypointAction'
            })

            this.goal = new ROSLIB.Goal({
                actionClient: actionClient,
                goalMessage: this.action.goal,
            })

            this.goal.on('status', (status) => {
                this.action.status = status
            })

            this.goal.on('feedback', (feedback) => {
                this.action.feedback = feedback
            })

            this.goal.on('result', (result) => {
                this.action.result = result
            })

            this.goal.send()

        },
        cancelGoal: function(){
            this.goal.cancel()
        },

        sendGoalPos1: function(){
        
            this.action.goal.position.x = 0.6651526761626221
            this.action.goal.position.y = -0.4795577157970588
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
        sendGoalPos2: function(){
        
            this.action.goal.position.x = 0.6562840738349672
            this.action.goal.position.y = 0.48647307167704706
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
        sendGoalPos3: function(){
        
            this.action.goal.position.x = 0.23087315927409957
            this.action.goal.position.y = 0.47514464201419954
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
        sendGoalPos4: function(){
        
            this.action.goal.position.x = 0.19143743535213542
            this.action.goal.position.y = 0.03457620028720394
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
        sendGoalPos5: function(){
        
            this.action.goal.position.x = -0.1133238491417585
            this.action.goal.position.y = -0.013586357332577697
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
        sendGoalPos6: function(){
        
            this.action.goal.position.x = -0.1810594112125771
            this.action.goal.position.y = 0.4703493204506495
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
        sendGoalPos7: function(){
        
            this.action.goal.position.x = -0.6231508246917589
            this.action.goal.position.y = 0.47637329817159724
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
        sendGoalPos8: function(){
        
            this.action.goal.position.x = -0.18853543095858388
            this.action.goal.position.y = -0.48416041763764833
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
        sendGoalPos9: function(){
        
            this.action.goal.position.x = -0.48304228364701696
            this.action.goal.position.y = -0.5295696157948706
            this.action.goal.position.z = 0
            this.sendGoal()
        
        },
    },
    mounted() {
        // page is ready
        window.addEventListener('mouseup', this.stopDrag)
        this.interval = setInterval(() => {
            if (this.ros != null && this.ros.isConnected) {
                this.ros.getNodes((data) => { }, (error) => { })
            }
        }, 10000)
    },
})