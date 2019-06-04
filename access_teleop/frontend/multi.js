/**
 * Created by timadamson on 8/22/17.
 */

/**
 * An App self adds 3 camera streams, and the controlers to move the robot
 */

// Number of the current task
var taskNum;

function init() {
    var arm_div = document.querySelectorAll('.js_arm_div');
    this.app = new App();
    var self = this;

    app.ros.on('connection', function () {
        app.advertiseAll();
        console.log("We are connected!");
    });

    app.ros.on('close', function(error) {
       app.unadvertiseAll();
       console.error('We lost connection with ROS. All is lost');
    });

    var camera1 = document.getElementById("camera1");
    var camera2 = document.getElementById("camera2");    
    var camera3 = document.getElementById("camera3");

    var active_cam = "camera1";

    taskNum = 0;

    var titleWeb = document.getElementById("title");
    var titleWebH = titleWeb.offsetHeight;

    var btnTop = document.getElementById("top");

    var upBTN = document.createElement("button");
    upBTN.innerHTML = "<img src=\"img/blueUpArrow.png\">";
    upBTN.id = 'up';
    upBTN.title = '\u2191Up\u2191';
    var body = document.getElementsByTagName("body")[0];
    body.appendChild(upBTN);

    var downBTN = document.createElement("button");
    downBTN.innerHTML = "<img src=\"img/blueDownArrow.png\">";
    downBTN.id = 'down';
    downBTN.title = '\u2193Down\u2193';
    downBTN.className = "button";
    body.appendChild(downBTN);

    var leftBTN = document.createElement("button");
    leftBTN.innerHTML = "<img src=\"img/greenLeftArrow.png\">";
    leftBTN.id = 'left';
    leftBTN.title = '\u2190Left\u2190';
    leftBTN.className = "button";
    leftBTN.style.width = 55 + "px";
    leftBTN.style.height = 80 + "px";
    body.appendChild(leftBTN);

    var rotateRightBTN = document.createElement("button");
    rotateRightBTN.innerHTML = "<img src=\"img/rotateClockwiseRight.png\">";
    rotateRightBTN.id = 'rotateRight';
    rotateRightBTN.title = '\u21BBRotate Right\u21BB';
    rotateRightBTN.className = "button";
    rotateRightBTN.style.width = 65 + "px"
    rotateRightBTN.style.height = 80 + "px";
    body.appendChild(rotateRightBTN);

    var rightBTN = document.createElement("button");
    rightBTN.innerHTML = "<img src=\"img/greenRightArrow.png\">";
    rightBTN.id = 'right';
    rightBTN.title = '\u2192Right\u2192';
    rightBTN.className = "button";
    rightBTN.style.width = 55 + "px";
    rightBTN.style.height = 80 + "px";
    body.appendChild(rightBTN);

    var rotateLeftBTN = document.createElement("button");
    rotateLeftBTN.innerHTML = "<img src=\"img/rotateAntiClockwiseLeft.png\">";
    rotateLeftBTN.id = 'rotateLeft';
    rotateLeftBTN.title = '\u21BARotate Left\u21BA';
    rotateLeftBTN.className = "button";
    rotateLeftBTN.style.width = 65 + "px";
    rotateLeftBTN.style.height = 80 + "px";
    body.appendChild(rotateLeftBTN);

    // Added by Xinyi
    var prevTaskBTN = document.getElementById("prevTask");
    var nextTaskBTN = document.getElementById("nextTask");
    var headUpBTN = document.getElementById("headUp");
    var headDownBTN = document.getElementById("headDown");

    // mouse down
    upBTN.addEventListener("mousedown", function () {
		app.arm.moveArmByDelta("0", "-10", active_cam);
    });

    downBTN.addEventListener("mousedown", function () {
        app.arm.moveArmByDelta("0", "10", active_cam);
    });

    leftBTN.addEventListener("mousedown", function () {
        app.arm.moveArmByDelta("-10", "0", active_cam);
    });

    rightBTN.addEventListener("mousedown", function () {
        app.arm.moveArmByDelta("10", "0", active_cam);
    });

    rotateRightBTN.addEventListener("mousedown", function () {
    	if (active_cam == "camera1" || active_cam == "camera2") {
    		app.arm.orientByTheta(0.3, active_cam);
    	} else {
    		// rotates the gripper
    		app.wristRoller.rotate(-0.3);
    	}
        
    });

    rotateLeftBTN.addEventListener("mousedown", function () {
    	if (active_cam == "camera1" || active_cam == "camera2") {
        	app.arm.orientByTheta(-0.3, active_cam);
        } else  {
        	// rotates the gripper
        	app.wristRoller.rotate(0.3);
        }
    });

    prevTaskBTN.addEventListener("mousedown", function(e) {
		if (taskNum > 0) {
			app.base.goToPrev(taskNum);
			taskNum--;
		}
	});

	nextTaskBTN.addEventListener("mousedown", function(e) {
		if (taskNum < 5) {
			app.base.goToNext(taskNum);
			taskNum++;
		}
	});

	headUpBTN.addEventListener("mousedown", function() {
		app.head.tilt(0.2);
	});

	headDownBTN.addEventListener("mousedown", function() {
		app.head.tilt(-0.2);
	});

    // for continuous motion: mouse up
    // var btns = [upBTN, downBTN, leftBTN, rightBTN, rotateRightBTN, rotateLeftBTN];
    // for (var i = 0; i < btns.length; i++) {
    // 	btns[i].addEventListener("mouseup", function () {
    //     	clearInterval(this.baseCommand);
    // 	});
    // }

    $("#up").hide();
    $("#down").hide();
    $("#left").hide()
    $("#right").hide();
    $("#rotateRight").hide()
    $("#rotateLeft").hide();

    camera1.addEventListener("mouseover", showButtons);
    camera1.addEventListener("mouseout", hideButtons);
    camera2.addEventListener("mouseover", showButtons);
    camera2.addEventListener("mouseout", hideButtons);
    camera3.addEventListener("mouseover", showButtons);
    camera3.addEventListener("mouseout", hideButtons);

    function showButtons() {
    	showButtonsHelper(parseInt(this.id[6]));
    }

    function showButtonsHelper(cam_id) {
    	active_cam = "camera" + cam_id;
        $("#cam" + cam_id).css("color", "yellow");
        layoutButtons(cam_id, btnTop, upBTN, downBTN, leftBTN, rotateRightBTN, rightBTN, rotateLeftBTN); 
        $("#up").show();
        $("#down").show();
        $("#left").show();
        $("#right").show();
        $("#rotateRight").show();
        $("#rotateLeft").show();
    }

    function hideButtons() {
    	hideButtonsHelper(parseInt(this.id[6]));
    }

    function hideButtonsHelper(cam_id) {
    	$("#title" + cam_id).css("color", "blue");
        $("#up").hide();
        $("#down").hide();
        $("#left").hide();
        $("#right").hide()
        $("#rotateRight").hide()
        $("#rotateLeft").hide();
    }

    function layoutButtons(cameraId, btnTop, upBTN, downBTN, leftBTN, rotateRightBTN, rightBTN, rotateLeftBTN) {
    	var camera = document.getElementById("camera" + cameraId);

	    var btnTopH = btnTop.offsetHeight;
	    var btnUpH = camera.offsetHeight + titleWebH + btnTopH;
	    var offset = camera.offsetWidth * cameraId;
	    var divisor = 2 * cameraId;

	    upBTN.style.left = (offset - 95) / divisor * (divisor - 1) + "px";
        upBTN.style.top = (btnUpH) / 5 + "px";

    	downBTN.style.left = (offset - 95) / divisor * (divisor - 1) + "px";
    	downBTN.style.top = (btnUpH - titleWebH / 1.5) / 1 + "px";

    	leftBTN.style.left = camera.offsetWidth * (cameraId - 1) + "px";
    	leftBTN.style.top = (btnUpH) / 2 + "px";

    	rotateRightBTN.style.left = camera.offsetWidth * (cameraId - 1) + 55 + "px";
	    rotateRightBTN.style.top = (btnUpH) / 2 + "px";

	    rightBTN.style.left = offset - titleWebH - 30 + "px";
	    rightBTN.style.top = (btnUpH) / 2 + "px";

	    rotateLeftBTN.style.left = offset - titleWebH - 95 + "px";
	    rotateLeftBTN.style.top = (btnUpH) / 2 + "px";
    }

    init_flag = false;

}


function moveBase() {
	app.base.move("left");
}

function stopBase() {
	app.base.endBaseCommand();
}

