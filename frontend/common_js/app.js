/**
 * Created by timadamson on 8/22/17.
 */

/**
 * An App self adds 3 camera streams, and the controlers to move the robot
 */

App = function () {


    // Set self to be this so that you can add variables to this inside a callback
    var self = this;

    // Set up ros
    this.ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });


    this.ros.on('error', function(error) {
        console.log('Error connecting to websocket server.');
    });

    this.ros.on('close', function (error) {
       console.error('We lost connection with ROS. All is lost');
       document.body.innerHTML = "The connection with ROS is broken. Please reconnect";
    });

    this.arm = new Arm(this.ros);
    this.gripper = new Gripper(this.ros);
    this.cloudFreezer = new CloudFreezer(this.ros);
    this.wristRoller = new WristRoller(this.ros);

    //self.head = new Head(ros);

    // Set up the gripper event handlers
    // Calls itself after definition
    this.initRightClickGripper = function () {
        var arm_div = document.querySelectorAll('.js_arm_div');
        arm_div.forEach(function(element){
            element.addEventListener('contextmenu', function(ev){
                ev.preventDefault();
                if(self.gripper.getCurrentPosition() == self.gripper.PositionEnum.CLOSED ||
                    self.gripper.getCurrentPosition() == self.gripper.PositionEnum.PARTLY_CLOSED) {
                    self.gripper.open();
                }
                else {
                    self.gripper.close();
                }
               return false;
            }, false);
        });
    };

    this.addCloudFreezer = function(){
        var feedback = document.querySelector("#feedback");

        var freezeButton = document.createElement("button");
        freezeButton.innerHTML = "Freeze Point Cloud";
        freezeButton.onclick = this.cloudFreezer.freezeCloud;

        var unfreezeButton = document.createElement("button");
        unfreezeButton.innerHTML = "Real Time Point Cloud";
        unfreezeButton.onclick = this.cloudFreezer.unfreezeCloud;

        feedback.appendChild(freezeButton);
        feedback.appendChild(unfreezeButton);
    };

    this.moveLine = function (x1,y1, x2, y2, camera_name) {
        if(camera_name) {
            var line = document.querySelector("#" + camera_name + "  svg line");
            line.setAttribute('x1', x1);
            line.setAttribute('y1', y1);
            line.setAttribute('x2', x2);
            line.setAttribute('y2', y2);
        }
    };

    this.moveArrow = function (x1,y1, x2, y2, camera_name) {
        if(camera_name) {
            var polyline = document.querySelector("#" + camera_name + "  svg polyline");
            polyline.setAttribute('points', `${x1},${y1} ${x2},${y2}`);
        }
    };

    if(!self.handleStatus){
        this.handleStatus = function (message) {
            console.log("Generaic handler called");
        };
        console.log("Using the app's arm handler");
    }


// This is the double slider for the gripper
    $(document).arrive("#slider-range", function () {
        $( function() {
            $slider = $( "#slider-range" ); //This will create the slider jQuery object as soon as the element is ready
            app.gripper.gripperGUI = new GripperGUI(app.gripper); //This has to be here, because otherwise the element wil not be loaded
        });
    });

    // Adds 3 canvas image streams
    // --------------------------------------------------------------------------------
    // Dynamic Canvas Sizes
    //var camCanvas = document.getElementById("cam1");




    // Dynamic Canvas Sizes

    var elFirstSt =0;
    var elTwoSt =0;
    var elThreeSt =0;

    var cam1Container = document.getElementById("cam1-container");
    var cam2Container = document.getElementById("cam2-container");
    var cam3Container = document.getElementById("cam3-container");


    var dimCam1Width = cam1Container.clientWidth;
    var dimCam1Height = cam1Container.clientWidth;//dimCam1.clientHeight;

    var dimCam2Width = cam2Container.clientWidth;
    var dimCam2Height = cam2Container.clientWidth;//dimCam2.clientHeight;

    var dimCam3Width = cam3Container.clientWidth;
    var dimCam3Height = cam3Container.clientWidth;//dimCam2.clientHeight;

    this.cameraWidth = Math.max(dimCam1Width, dimCam2Width); //This will get the size of the container
    this.cameraHeight = this.cameraWidth;

    this.backendCameraWidth = "640";
    this.backendCameraHeight = "480";

    //document.getElementById("camera1").style.height = this.cameraHeight + 'px';
    //document.getElementById("camera1").style.width = this.cameraWidth + 'px';

    //document.getElementById("camera2").style.height = this.cameraHeight + 'px';
    //document.getElementById("camera2").style.width = this.cameraWidth + 'px';


    // Create the main viewer.
    var viewer1 = new MJPEGCANVAS.Viewer({
        divID : 'camera1',
        host : 'localhost',
        width : dimCam1Width,
        height : dimCam1Height,
        topic : '/rviz1/camera1/image'
    });

    // Create the main viewer.
    var viewer2 = new MJPEGCANVAS.Viewer({
        divID : 'camera2',
        host : 'localhost',
        width : dimCam2Width,
        height : dimCam2Height,
        topic : '/rviz1/camera2/image'
    });

    // Create the main viewer.
    var viewer3= new MJPEGCANVAS.Viewer({
        divID : 'camera3',
        host : 'localhost',
        width : dimCam3Width,
        height : dimCam3Height,
        topic : '/head_camera/rgb/image_raw'
    });

    // ------------------------------------------------------------------------------------------

    init_flag = false;


    function hidePanel() {
        var expr = this.id;

        switch (expr){
            case 'cameraTop':
                elFirstSt =1;
                cam1Container.classList.add("col-sm-1");
                cam1Container.classList.remove("col-sm-10");
                cam1Container.classList.remove("col-sm-5");
                cam1Container.classList.remove("col-sm-4");

                // other panels
                if(elTwoSt ==0 && elThreeSt==0){
                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");

                    dimCam2Width = cam2Container.clientWidth;
                    dimCam3Width = cam3Container.clientWidth;
                    viewer2.canvas.width= dimCam2Width;
                    viewer3.canvas.width= dimCam3Width;
                    viewer2.width=dimCam2Width;
                    viewer3.width=dimCam3Width;
                    //  viewer2.emit('resize');
                }else if(elTwoSt ==1 && elThreeSt==0 ){
                    cam3Container.classList.add("col-sm-10");
                    cam3Container.classList.remove("col-sm-5");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");

                    dimCam3Width = cam3Container.clientWidth;
                    viewer3.canvas.width= dimCam3Width;
                    viewer3.width=dimCam3Width;
                }else if(elTwoSt ==0 && elThreeSt==1){
                    cam2Container.classList.add("col-sm-10");
                    cam2Container.classList.remove("col-sm-5");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");

                    dimCam2Width = cam2Container.clientWidth;
                    viewer2.canvas.width= dimCam2Width;
                    viewer2.width=dimCam2Width;
                }else if(elTwoSt ==1 && elThreeSt==1 ){

                    //     alert('4');
                }
                break;
            case 'cameraSide':
                elTwoSt = 1;
                cam2Container.classList.add("col-sm-1");
                cam2Container.classList.remove("col-sm-10");
                cam2Container.classList.remove("col-sm-5");
                cam2Container.classList.remove("col-sm-4");

                // other panels
                if(elFirstSt ==0 && elThreeSt==0){
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");

                    dimCam1Width = cam1Container.clientWidth;
                    dimCam3Width = cam3Container.clientWidth;
                    viewer1.canvas.width= dimCam1Width;
                    viewer3.canvas.width= dimCam3Width;
                    viewer1.width=dimCam2Width;
                    viewer3.width=dimCam3Width;
                    //  viewer2.emit('resize');
                }else if(elFirstSt ==1 && elThreeSt==0 ){
                    cam3Container.classList.add("col-sm-10");
                    cam3Container.classList.remove("col-sm-5");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");

                    dimCam3Width = cam3Container.clientWidth;
                    viewer3.canvas.width= dimCam3Width;
                    viewer3.width=dimCam3Width;
                }else if(elFirstSt ==0 && elThreeSt==1){
                    cam1Container.classList.add("col-sm-10");
                    cam1Container.classList.remove("col-sm-5");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");

                    dimCam1Width =cam1Container.clientWidth;
                    viewer1.canvas.width= dimCam1Width;
                    viewer1.width=dimCam1Width;
                }else if(elFirstSt ==1 && elThreeSt==1 ){

                    //     alert('4');
                }
                break;
            case 'cameraHead':
                elThreeSt =1;
                cam3Container.classList.add("col-sm-1");
                cam3Container.classList.remove("col-sm-10");
                cam3Container.classList.remove("col-sm-5");
                cam3Container.classList.remove("col-sm-4");

                // other panels
                if(elFirstSt ==0 && elTwoSt==0){
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");

                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");

                    dimCam1Width = cam1Container.clientWidth;
                    dimCam2Width = cam2Container.clientWidth;
                    viewer1.canvas.width= dimCam1Width;
                    viewer2.canvas.width= dimCam2Width;
                    viewer1.width=dimCam1Width;
                    viewer2.width=dimCam2Width;
                    //  viewer2.emit('resize');
                }else if(elFirstSt ==1 && elTwoSt==0 ){
                    cam2Container.classList.add("col-sm-10");
                    cam2Container.classList.remove("col-sm-5");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");

                    dimCam2Width = cam2Container.clientWidth;
                    viewer2.canvas.width= dimCam2Width;
                    viewer2.width=dimCam2Width;
                }else if(elFirstSt ==0 && elTwoSt==1){
                    cam1Container.classList.add("col-sm-10");
                    cam1Container.classList.remove("col-sm-5");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");

                    dimCam1Width =cam1Container.clientWidth;
                    viewer1.canvas.width= dimCam1Width;
                    viewer1.width=dimCam1Width;
                }else if(elFirstSt ==1 && elTwoSt==1 ){

                    //     alert('4');
                }
                break;
        }
    }


    function shownPanel() {
        var expr = this.id;

        switch (expr){
            case 'cameraTop':
                elFirstSt =0;

                // other panels
                if(elTwoSt ==0 && elThreeSt==0){
                    cam1Container.classList.add("col-sm-4");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-5");
                    cam1Container.classList.remove("col-sm-1");

                    cam2Container.classList.add("col-sm-4");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-5");
                    cam2Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-4");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-5");
                    cam3Container.classList.remove("col-sm-1");

                    //  viewer2.emit('resize');
                }else if(elTwoSt ==1 && elThreeSt==0 ){
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");

                }else if(elTwoSt ==0 && elThreeSt==1){
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");

                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");

                }else if(elTwoSt ==1 && elThreeSt==1 ){
                    cam1Container.classList.add("col-sm-10");
                    cam1Container.classList.remove("col-sm-5");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");
                }


                break;
            case 'cameraSide':
                elTwoSt = 0;

                // other panels
                if(elFirstSt ==0 && elThreeSt==0){
                    cam1Container.classList.add("col-sm-4");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-5");
                    cam1Container.classList.remove("col-sm-1");

                    cam2Container.classList.add("col-sm-4");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-5");
                    cam2Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-4");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-5");
                    cam3Container.classList.remove("col-sm-1");
                }else if(elFirstSt ==1 && elThreeSt==0 ){
                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");

                }else if(elFirstSt ==0 && elThreeSt==1){
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");

                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");

                }else if(elFirstSt ==1 && elThreeSt==1 ){
                    cam2Container.classList.add("col-sm-10");
                    cam2Container.classList.remove("col-sm-5");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");
                }
                break;
            case 'cameraHead':;
                elThreeSt =0;

                // other panels
                if(elFirstSt ==0 && elTwoSt==0){
                    cam1Container.classList.add("col-sm-4");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-5");
                    cam1Container.classList.remove("col-sm-1");

                    cam2Container.classList.add("col-sm-4");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-5");
                    cam2Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-4");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-5");
                    cam3Container.classList.remove("col-sm-1");
                }else if(elFirstSt ==1 && elTwoSt==0 ){
                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10");
                    cam2Container.classList.remove("col-sm-4");
                    cam2Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");

                }else if(elFirstSt ==0 && elTwoSt==1){
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10");
                    cam1Container.classList.remove("col-sm-4");
                    cam1Container.classList.remove("col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");

                }else if(elFirstSt ==1 && elTwoSt==1 ){
                    cam3Container.classList.add("col-sm-10");
                    cam3Container.classList.remove("col-sm-5");
                    cam3Container.classList.remove("col-sm-4");
                    cam3Container.classList.remove("col-sm-1");
                }
                break;
        }

        dimCam1Width =cam1Container.clientWidth;
        dimCam2Width = cam2Container.clientWidth;
        dimCam3Width = cam3Container.clientWidth;
        viewer1.canvas.width= dimCam1Width;
        viewer2.canvas.width= dimCam2Width;
        viewer3.canvas.width= dimCam3Width;
        viewer1.width=dimCam1Width;
        viewer2.width=dimCam2Width;
        viewer3.width=dimCam3Width;
    }
    $(window).resize(function(){
        dimCam1Width = cam1Container.clientWidth;
        dimCam2Width = cam2Container.clientWidth;
        dimCam3Width = cam3Container.clientWidth;
        viewer1.canvas.width= dimCam1Width;
        viewer2.canvas.width= dimCam2Width;
        viewer3.canvas.width= dimCam3Width;
        viewer1.width=dimCam1Width;
        viewer2.width=dimCam2Width;
        viewer3.width=dimCam3Width;
    });

    $("#cameraTop").on('shown.bs.collapse', shownPanel);
    $("#cameraSide").on('shown.bs.collapse', shownPanel);
    $("#cameraHead").on('shown.bs.collapse', shownPanel);

    $("#cameraTop").on('hidden.bs.collapse', hidePanel);
    $("#cameraSide").on('hidden.bs.collapse', hidePanel);
    $("#cameraHead").on('hidden.bs.collapse', hidePanel);
};



