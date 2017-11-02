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
//       document.body.innerHTML = "The connection with ROS is broken. Please reconnect";
    });
    /*
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
        };*/


    if(!self.handleStatus){
        this.handleStatus = function (message) {
            console.log("Generaic handler called");
        };
        console.log("Using the app's arm handler");
    }

    /*

    // This is the double slider for the gripper
        $(document).arrive("#slider-range", function () {
            $( function() {
                $slider = $( "#slider-range" ); //This will create the slider jQuery object as soon as the element is ready
                app.gripper.gripperGUI = new GripperGUI(app.gripper); //This has to be here, because otherwise the element wil not be loaded
            });
        });*/

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
    var dimCam1Height = cam1Container.clientWidth / aspectRatio;

    var dimCam2Width = cam2Container.clientWidth;
    var dimCam2Height = cam2Container.clientWidth / aspectRatio;

    var dimCam3Width = cam3Container.clientWidth;
    var dimCam3Height = cam3Container.clientWidth / aspectRatio;

    this.backendCameraWidth = "640";
    this.backendCameraHeight = "480";

    var aspectRatio = this.backendCameraWidth/this.backendCameraHeight;

    this.dimCam1Width =  cam1Container.clientWidth;
    this.dimCam1Height =cam1Container.clientWidth/aspectRatio;

    this.dimCam2Width = cam2Container.clientWidth;
    this.dimCam2Height = cam2Container.clientWidth/aspectRatio;
    this.dimCam1Height = cam1Container.clientWidth/aspectRatio;

    this.dimCam2Width = cam2Container.clientWidth;
    this.dimCam2Height = cam2Container.clientWidth/aspectRatio;

    this.dimCam3Width = cam3Container.clientWidth;
    this.dimCam3Height = cam3Container.clientWidth/aspectRatio;

    this.cameraWidth = Math.max(this.dimCam1Width,this.dimCam2Width); //This will get the size of the container
    this.cameraHeight = this.cameraWidth/aspectRatio;


    var elTitle = document.getElementById("title");
    var elTopBTN = document.getElementById("topBTN");
    var elcmdReceived = document.getElementById("cmdReceived");
    var dimcmdReceivedHeight = elcmdReceived.clientHeight;

    var dimTitleHeight = elTitle.offsetHeight;
    var dimTopBTNHeight = elTopBTN.offsetHeight;

    // Create the main viewer.
    var viewer1 = new MJPEGCANVAS.Viewer({
        divID : 'camera1',
        host : 'localhost',
        width :this.dimCam1Width,
        height :this.dimCam1Height,
        topic : '/rviz1/camera1/image'
    });

    // Create the main viewer.
    var viewer2 = new MJPEGCANVAS.Viewer({
        divID : 'camera2',
        host : 'localhost',
        width :this.dimCam2Width,
        height :this.dimCam2Height,
        topic : '/rviz1/camera2/image'
    });

    // Create the main viewer.
    var viewer3= new MJPEGCANVAS.Viewer({
        divID : 'camera3',
        host : 'localhost',
        width :this.dimCam3Width,
        height :this.dimCam3Height,
        topic : '/head_camera/rgb/image_raw'
    });
    var originaldimCam1Width = viewer1.width;

    var originaldimCam1Height =viewer1.canvas.height;

    var originaldimCam2Width = viewer2.canvas.width;

    var originaldimCam2Height =viewer2.canvas.height;

    var originaldimCam3Width = viewer3.canvas.width;

    var originaldimCam3Height =viewer3.canvas.height;

    //alert (viewer1.width);    ------------------------------------------------------------------------------------------

    var camera1 = document.getElementById("camera1");
    var camera2 = document.getElementById("camera2");
    var camera3 = document.getElementById("camera3");
    var elTopBTN= document.getElementById("topBTN");
    var dimTopBTNHeight = elTopBTN.clientHeight;

    var topBTN = document.getElementById("topBTN");
    var sideBTN = document.getElementById("sideBTN");
    var headBTN = document.getElementById("headBTN");
    var svgCam1 = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    svgCam1.id='svgCam1';
    svgCam1.setAttribute('style', 'top: '+dimTopBTNHeight+'px;border: 1px solid black;background: transparent;position:inherit;');
    svgCam1.setAttribute('width',this.dimCam1Width);
    svgCam1.setAttribute('height',this.dimCam1Height);
    svgCam1.setAttributeNS("http://www.w3.org/2000/xmlns/", "xmlns:xlink", "http://www.w3.org/1999/xlink");

    var svgOverlay1 = document.getElementById("camera1");
    //  svgOverlay1.appendChild(svgCam1);

    var s = Snap("#svgCam1");
    var linex1 = 20;
    var liney1 = 15;
    var linex2 = 50;
    var liney2 = 50;
    /*

     var x,y;
        var line = s.line(linex1,liney1,linex2,liney2);
        line.attr({
          stroke: "#008000",
          strokeWidth: 10
        });
        var radius = 10;

        var circle1 = s.circle(linex1, liney1, radius).attr({ fill: "red" });
        var circle2 = s.circle(linex2, liney2, radius).attr({ fill: "green" });
    */
    function moveFunc( ev, x, y ) {

        var coor = "(" + x + "," + y + ")";
        //  document.getElementById("coordReceived").innerHTML = coor;
        // circle2.attr({ cx: x, cy: y-25});

    };
    /*s.click(moveFunc);
    s.unmouseover(moveFunc);

    s.mouseover(moveFunc);
        var svgElement = document.getElementById("svgCam1");
        svgElement.addEventListener("mouseover", mouseOver);

        svgElement.style.position = "absolute";
        function mouseOver() {
          // alert(this.id);
             // x = circle2.attr("cx");
       // y = circle2.attr("cy");
      //  line.attr({x2:7,y2:9});
        }*/
    init_flag = false;


    function hidePanel() {
        var expr = this.id;

        switch (expr) {
            case 'cameraTop':
                elFirstSt = 1;
                cam1Container.classList.add("col-sm-1");
                cam1Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-4");

                // other panels
                if (elTwoSt == 0 && elThreeSt == 0) {
                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    dimCam2Width = cam2Container.clientWidth;
                    dimCam3Width = cam3Container.clientWidth;
                    viewer2.canvas.width = dimCam2Width;
                    viewer3.canvas.width = dimCam3Width;
                    viewer2.width = dimCam2Width;
                    viewer3.width = dimCam3Width;
                    //  viewer2.emit('resize');
                } else if (elTwoSt == 1 && elThreeSt == 0) {
                    cam3Container.classList.add("col-sm-10");
                    cam3Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");

                    dimCam3Width = cam3Container.clientWidth;
                    viewer3.canvas.width = dimCam3Width;
                    viewer3.width = dimCam3Width;
                } else if (elTwoSt == 0 && elThreeSt == 1) {
                    cam2Container.classList.add("col-sm-10");
                    cam2Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");

                    dimCam2Width = cam2Container.clientWidth;
                    viewer2.canvas.width = dimCam2Width;
                    viewer2.width = dimCam2Width;
                } else if (elTwoSt == 1 && elThreeSt == 1) {

                    //     alert('4');
                }
                break;
            case 'cameraSide':
                elTwoSt = 1;
                cam2Container.classList.add("col-sm-1");
                cam2Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-4");

                // other panels
                if (elFirstSt == 0 && elThreeSt == 0) {
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    dimCam1Width = cam1Container.clientWidth;
                    dimCam3Width = cam3Container.clientWidth;
                    viewer1.canvas.width = dimCam1Width;
                    viewer3.canvas.width = dimCam3Width;
                    viewer1.width = dimCam2Width;
                    viewer3.width = dimCam3Width;
                    //  viewer2.emit('resize');
                } else if (elFirstSt == 1 && elThreeSt == 0) {
                    cam3Container.classList.add("col-sm-10");
                    cam3Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");

                    dimCam3Width = cam3Container.clientWidth;
                    viewer3.canvas.width = dimCam3Width;
                    viewer3.width = dimCam3Width;
                } else if (elFirstSt == 0 && elThreeSt == 1) {
                    cam1Container.classList.add("col-sm-10");
                    cam1Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");

                    dimCam1Width = cam1Container.clientWidth;
                    viewer1.canvas.width = dimCam1Width;
                    viewer1.width = dimCam1Width;
                } else if (elFirstSt == 1 && elThreeSt == 1) {

                    //     alert('4');
                }
                break;
            case 'cameraHead':
                elThreeSt = 1;
                cam3Container.classList.add("col-sm-1");
                cam3Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-4");

                // other panels
                if (elFirstSt == 0 && elTwoSt == 0) {
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    dimCam1Width = cam1Container.clientWidth;
                    dimCam2Width = cam2Container.clientWidth;
                    viewer1.canvas.width = dimCam1Width;
                    viewer2.canvas.width = dimCam2Width;
                    viewer1.width = dimCam1Width;
                    viewer2.width = dimCam2Width;
                    //  viewer2.emit('resize');
                } else if (elFirstSt == 1 && elTwoSt == 0) {
                    cam2Container.classList.add("col-sm-10");
                    cam2Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");

                    dimCam2Width = cam2Container.clientWidth;
                    viewer2.canvas.width = dimCam2Width;
                    viewer2.width = dimCam2Width;
                } else if (elFirstSt == 0 && elTwoSt == 1) {
                    cam1Container.classList.add("col-sm-10");
                    cam1Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");

                    dimCam1Width = cam1Container.clientWidth;
                    viewer1.canvas.width = dimCam1Width;
                    viewer1.width = dimCam1Width;
                } else if (elFirstSt == 1 && elTwoSt == 1) {

                    //     alert('4');
                }
                break;
        }

        dimCam1Width = cam1Container.clientWidth;
        dimCam2Width = cam2Container.clientWidth;
        dimCam3Width = cam3Container.clientWidth;
        dimCam1Height = dimCam1Width / aspectRatio;
        dimCam2Height = dimCam2Width / aspectRatio;
        dimCam3Height = dimCam3Width / aspectRatio;

        viewer1.canvas.width = dimCam1Width;
        viewer2.canvas.width = dimCam2Width;
        viewer3.canvas.width = dimCam3Width;
        viewer1.width = dimCam1Width;
        viewer2.width = dimCam2Width;
        viewer3.width = dimCam3Width;
        viewer1.canvas.height = viewer1.height = dimCam1Height;
        viewer2.canvas.height = viewer2.height = dimCam2Height;
        viewer3.canvas.height = viewer3.height = dimCam3Height;


        resizeWindow();
    }


    function shownPanel() {
        var expr = this.id;

        switch (expr) {
            case 'cameraTop':
                elFirstSt = 0;

                // other panels
                if (elTwoSt == 0 && elThreeSt == 0) {
                    cam1Container.classList.add("col-sm-4");
                    cam1Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");

                    cam2Container.classList.add("col-sm-4");
                    cam2Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");

                    cam3Container.classList.add("col-sm-4");
                    cam3Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");

                    //  viewer2.emit('resize');
                } else if (elTwoSt == 1 && elThreeSt == 0) {
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                } else if (elTwoSt == 0 && elThreeSt == 1) {
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                } else if (elTwoSt == 1 && elThreeSt == 1) {
                    cam1Container.classList.add("col-sm-10");
                    cam1Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");
                }


                break;
            case 'cameraSide':
                elTwoSt = 0;

                // other panels
                if (elFirstSt == 0 && elThreeSt == 0) {
                    cam1Container.classList.add("col-sm-4");
                    cam1Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");

                    cam2Container.classList.add("col-sm-4");
                    cam2Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");

                    cam3Container.classList.add("col-sm-4");
                    cam3Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");

                } else if (elFirstSt == 1 && elThreeSt == 0) {
                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                } else if (elFirstSt == 0 && elThreeSt == 1) {
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                } else if (elFirstSt == 1 && elThreeSt == 1) {
                    cam2Container.classList.add("col-sm-10");
                    cam2Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");
                }
                break;
            case 'cameraHead':
                ;
                elThreeSt = 0;

                // other panels
                if (elFirstSt == 0 && elTwoSt == 0) {
                    cam1Container.classList.add("col-sm-4");
                    cam1Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");

                    cam2Container.classList.add("col-sm-4");
                    cam2Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");

                    cam3Container.classList.add("col-sm-4");
                    cam3Container.classList.remove("col-sm-10", "col-sm-5", "col-sm-1");
                } else if (elFirstSt == 1 && elTwoSt == 0) {
                    cam2Container.classList.add("col-sm-5");
                    cam2Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");


                } else if (elFirstSt == 0 && elTwoSt == 1) {
                    cam1Container.classList.add("col-sm-5");
                    cam1Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");

                    cam3Container.classList.add("col-sm-5");
                    cam3Container.classList.remove("col-sm-10", "col-sm-4", "col-sm-1");


                } else if (elFirstSt == 1 && elTwoSt == 1) {
                    cam3Container.classList.add("col-sm-10");
                    cam3Container.classList.remove("col-sm-5", "col-sm-4", "col-sm-1");
                }
                break;
        }

        dimCam1Width = cam1Container.clientWidth;
        dimCam2Width = cam2Container.clientWidth;
        dimCam3Width = cam3Container.clientWidth;
        dimCam1Height = dimCam1Width / aspectRatio;
        dimCam2Height = dimCam2Width / aspectRatio;
        dimCam3Height = dimCam3Width / aspectRatio;

        viewer1.canvas.width = dimCam1Width;
        viewer2.canvas.width = dimCam2Width;
        viewer3.canvas.width = dimCam3Width;
        viewer1.width = dimCam1Width;
        viewer2.width = dimCam2Width;
        viewer3.width = dimCam3Width;
        viewer1.canvas.height = viewer1.height = dimCam1Height;
        viewer2.canvas.height = viewer2.height = dimCam2Height;
        viewer3.canvas.height = viewer3.height = dimCam3Height;

        resizeWindow();
    }



    /*    $(window).resize(function () {
            winHeight = parseInt($(window).height());
            winWidth = parseInt($(window).width());
            document.getElementById("cmdReceived").innerHTML = " w=" + winWidth + " h=" + winHeight;
            if (winWidth < 600) {
                dimCam1Height = winHeight / 3 - dimTitleHeight - dimTopBTNHeight;//winHeight/3;//dimCam1Width/aspectRatio;
                dimCam2Height = winHeight / 3 - dimTitleHeight - dimTopBTNHeight;//dimCam2Width/aspectRatio;
                dimCam3Height = winHeight / 3 - dimTitleHeight - dimTopBTNHeight;
                dimCam1Width = dimCam1Height * aspectRatio;//winWidth/3;//cam1Container.clientWidth;
                dimCam2Width = dimCam2Height * aspectRatio;//winWidth/3;//cam2Container.clientWidth;
                dimCam3Width = dimCam3Height * aspectRatio;
            } else {
                dimCam1Width = winWidth / 3;//cam1Container.clientWidth;
                dimCam2Width = winWidth / 3;//cam2Container.clientWidth;
                dimCam3Width = winWidth / 3;//cam3Container.clientWidth;
                dimCam1Height = dimCam1Width / aspectRatio;
                dimCam2Height = dimCam2Width / aspectRatio;
                dimCam3Height = dimCam3Width / aspectRatio;
            }

          //  alert (dimCam1Width);
            this.dimCam1Width=dimCam1Width;
            this.originaldimCam1Width= viewer1.canvas.width;
            viewer1.width = viewer1.canvas.width = dimCam1Width;
            this.dimCam2Width=viewer2.width = viewer2.canvas.width = dimCam2Width;
            this.dimCam3Width  =viewer3.width = viewer3.canvas.width = dimCam3Width;

            this.dimCam1Height=viewer1.canvas.height = viewer1.height = dimCam1Height;
            this.dimCam2Height=viewer2.canvas.height = viewer2.height = dimCam2Height;
            this.dimCam3Height=viewer3.canvas.height = viewer3.height = dimCam3Height;

        });*/
    /*    $(window).on('resize', _.debounce(function() {


            //  =       console.clear();
            console.log("new2 ="+this.dimCam1Width);
        //    alert ("okay2 ="+ this.dimCam1Width);
        }, 400));*/

// Helper function to get an element's exact position
    function getPosition(el) {
        var xPos = 0;
        var yPos = 0;

        while (el) {
            if (el.tagName == "BODY") {
                // deal with browser quirks with body/window/document and page scroll
                var xScroll = el.scrollLeft || document.documentElement.scrollLeft;
                var yScroll = el.scrollTop || document.documentElement.scrollTop;

                xPos += (el.offsetLeft - xScroll + el.clientLeft);
                yPos += (el.offsetTop - yScroll + el.clientTop);
            } else {
                // for all other non-BODY elements
                xPos += (el.offsetLeft - el.scrollLeft + el.clientLeft);
                yPos += (el.offsetTop - el.scrollTop + el.clientTop);
            }

            el = el.offsetParent;
        }
        return {
            x: xPos,
            y: yPos
        };
    }

// deal with the page getting resized or scrolled
    window.addEventListener("scroll", updatePosition, false);
    window.addEventListener("resize", updatePosition, false);

    function updatePosition() {
        // add your code to update the position when your browser
        // is resized or scrolled
    }
    var myEfficientFn = _.debounce(function() {
        resizeWindow();/*
        console.log("deb ="+originaldimCam1Width);
        var controls1 = document.getElementById("controls1");
        var controls2 = document.getElementById("controls2");
        var ratio =dimCam1Width/originaldimCam1Width;

        //  ; //Left side of box
        controls2.offsetTop;  //Top side of box
        controls2.offsetLeft + controls2.offsetWidth; //Right side of box
        controls2.offsetTop + controls2.offsetHeight; //Bottom side of box
        controls1.style.top=-dimCam1Height-7+ 'px';
        controls2.style.transform='scale('+ratio+','+ratio+')';
        controls2.style.top=-dimCam2Height-7+ 'px';
        //    controls2.style.transform='translate('+7+ 'px'+','+7+ 'px'+')';
        var position = getPosition(controls2);
        var pos2 = getPosition(cam2Container);
        controls2.style.left=-(sideBTN.offsetLeft)+"px";
        controls1.style.transform='scale('+ratio+','+ratio+')';
        //sideBTN      var myElement = document.querySelector("#foo");
        alert("The image is located at: " + position.x + ", " +controls2.offsetLeft );
        //  (50px, 100px)  // scaleconsole.clear();
        console.log("original ="+originaldimCam1Width);
        console.log("new ="+dimCam1Width);
        console.log("ratio ="+ratio);*/
        //   location.reload();
    }, 400);

    window.addEventListener('resize', myEfficientFn);
    function resizeWindow() {

        var elcmdReceived = document.getElementById("speechControl");
        var dimcmdReceivedHeight = elcmdReceived.clientHeight;
        var height = window.innerHeight || document.documentElement.clientHeight || document.body.clientHeight;
        var scrollHeight = document.documentElement.scrollHeight;

        if (scrollHeight <= height) {
        } else {
            if (cam1Container.classList.contains("col-sm-10") || cam2Container.classList.contains("col-sm-10") || cam3Container.classList.contains("col-sm-10")) {
                dimCam1Height = dimCam1Height - (scrollHeight - height) + dimTitleHeight + dimcmdReceivedHeight;
                dimCam2Height = dimCam2Height - (scrollHeight - height) + dimTitleHeight + dimcmdReceivedHeight;
                dimCam3Height = dimCam3Height - (scrollHeight - height) + dimTitleHeight + dimcmdReceivedHeight;
            } else {
                dimCam1Height = dimCam1Height - (scrollHeight - height) + dimcmdReceivedHeight;
                dimCam2Height = dimCam2Height - (scrollHeight - height) + dimcmdReceivedHeight;
                dimCam3Height = dimCam3Height - (scrollHeight - height) + dimcmdReceivedHeight;

            }

            dimCam1Width = dimCam1Height * aspectRatio;
            dimCam2Width = dimCam2Height * aspectRatio;
            dimCam3Width = dimCam3Height * aspectRatio;

        }
        winHeight = parseInt($(window).height());
        winWidth = parseInt($(window).width());
        document.getElementById("cmdReceived").innerHTML = " w=" + winWidth + " h=" + winHeight;
        if (winWidth < 600) {
            dimCam1Height = winHeight / 3 - dimTitleHeight - dimTopBTNHeight;//winHeight/3;//dimCam1Width/aspectRatio;
            dimCam2Height = winHeight / 3 - dimTitleHeight - dimTopBTNHeight;//dimCam2Width/aspectRatio;
            dimCam3Height = winHeight / 3 - dimTitleHeight - dimTopBTNHeight;// dimCam3Width/aspectRatio;

            dimCam1Width = dimCam1Height * aspectRatio;//winWidth/3;//cam1Container.clientWidth;
            dimCam2Width = dimCam2Height * aspectRatio;//winWidth/3;//cam2Container.clientWidth;
            dimCam3Width = dimCam3Height * aspectRatio;//winWidth/3;//cam3Container.clientWidth;

        } else {
            dimCam1Width = winWidth / 3;//cam1Container.clientWidth;
            dimCam2Width = winWidth / 3;//cam2Container.clientWidth;
            dimCam3Width = winWidth / 3;//cam3Container.clientWidth;
            dimCam1Height = dimCam1Width / aspectRatio;
            dimCam2Height = dimCam2Width / aspectRatio;
            dimCam3Height = dimCam3Width / aspectRatio;
        }

        viewer1.width =  viewer1.canvas.width =dimCam1Width;
        this.dimCam1Width= dimCam1Width;
        this.dimCam2Width=viewer2.width = viewer2.canvas.width = dimCam2Width;
        this.dimCam3Width  =viewer3.width = viewer3.canvas.width = dimCam3Width;

        this.dimCam1Height=viewer1.canvas.height = viewer1.height = dimCam1Height;
        this.dimCam2Height=viewer2.canvas.height = viewer2.height = dimCam2Height;
        this.dimCam3Height=viewer3.canvas.height = viewer3.height = dimCam3Height;


    }
    this.getCam1W = function(){
        return dimCam1Width;
    }
    this.getCam1H = function(){
        return dimCam1Width;
    }

    this.getCam2W = function(){
        return dimCam2Width;
    }

    this.getCam2H = function(){
        return dimCam2Height;
    }

    this.getCam1WO = function(){
        return originaldimCam1Width;
    }
    this.getCam2WO = function(){
        return originaldimCam2Width;
    }

    $("#cameraTop").on('shown.bs.collapse', shownPanel);
    $("#cameraSide").on('shown.bs.collapse', shownPanel);
    $("#cameraHead").on('shown.bs.collapse', shownPanel);

    $("#cameraTop").on('hidden.bs.collapse', hidePanel);
    $("#cameraSide").on('hidden.bs.collapse', hidePanel);
    $("#cameraHead").on('hidden.bs.collapse', hidePanel);



};



