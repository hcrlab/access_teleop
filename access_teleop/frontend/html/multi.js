/**
 * Created by timadamson on 8/22/17.
 */

/**
 * An App self adds 3 camera streams, and the controlers to move the robot
 */

// Number of cameras
var camNum = 3;
// Number of the current task
var taskNum;
// button dimensions
var VERTICAL_BTN_H = 80;
var VERTICAL_BTN_W = 55;
var VERTICAL_BTN_W_WIDE = 65;
var HORIZONTAL_BTN_H = 55;
var HORIZONTAL_BTN_W = 80;

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

    var cameras = [];
    for (var i = 0; i < camNum; i++) {
        cameras.push(document.getElementById("camera" + (i + 1)));
    }

    var active_cam = "camera1";

    taskNum = 0;

    var titleWeb = document.getElementById("title");
    var titleWebH = titleWeb.offsetHeight;

    var btnTop = document.getElementById("top");

    var upBTN = document.createElement("button");
    upBTN.innerHTML = "<img src=\"img/blueUpArrow.png\">";
    upBTN.id = 'up';
    upBTN.title = '\u2191Up\u2191';

    var downBTN = document.createElement("button");
    downBTN.innerHTML = "<img src=\"img/blueDownArrow.png\">";
    downBTN.id = 'down';
    downBTN.title = '\u2193Down\u2193';
    downBTN.className = "button";

    var leftBTN = document.createElement("button");
    leftBTN.innerHTML = "<img src=\"img/greenLeftArrow.png\">";
    leftBTN.id = 'left';
    leftBTN.title = '\u2190Left\u2190';
    leftBTN.className = "button";
    leftBTN.style.width = VERTICAL_BTN_W + "px";
    leftBTN.style.height = VERTICAL_BTN_H + "px";

    var rotateRightBTN = document.createElement("button");
    rotateRightBTN.innerHTML = "<img src=\"img/rotateClockwiseRight.png\">";
    rotateRightBTN.id = 'rotateRight';
    rotateRightBTN.title = '\u21BBRotate Right\u21BB';
    rotateRightBTN.className = "button";
    rotateRightBTN.style.width = VERTICAL_BTN_W_WIDE + "px"
    rotateRightBTN.style.height = VERTICAL_BTN_H + "px";

    var rightBTN = document.createElement("button");
    rightBTN.innerHTML = "<img src=\"img/greenRightArrow.png\">";
    rightBTN.id = 'right';
    rightBTN.title = '\u2192Right\u2192';
    rightBTN.className = "button";
    rightBTN.style.width = VERTICAL_BTN_W + "px";
    rightBTN.style.height = VERTICAL_BTN_H + "px";

    var rotateLeftBTN = document.createElement("button");
    rotateLeftBTN.innerHTML = "<img src=\"img/rotateAntiClockwiseLeft.png\">";
    rotateLeftBTN.id = 'rotateLeft';
    rotateLeftBTN.title = '\u21BARotate Left\u21BA';
    rotateLeftBTN.className = "button";
    rotateLeftBTN.style.width = VERTICAL_BTN_W_WIDE + "px";
    rotateLeftBTN.style.height = VERTICAL_BTN_H + "px";

    // Added by Xinyi
    var prevTaskBTN = document.getElementById("prevTask");
    var nextTaskBTN = document.getElementById("nextTask");
    var headUpBTN = document.getElementById("headUp");
    var headDownBTN = document.getElementById("headDown");
    var cloudBTN = document.getElementById("freeze");

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
    		app.arm.orientByTheta(0.2, active_cam);
    	} else {
    		// rotates the gripper
    		app.wristRoller.rotate(-0.2);
    	}
        
    });

    rotateLeftBTN.addEventListener("mousedown", function () {
    	if (active_cam == "camera1" || active_cam == "camera2") {
        	app.arm.orientByTheta(-0.2, active_cam);
        } else  {
        	// rotates the gripper
        	app.wristRoller.rotate(0.2);
        }
    });

    prevTaskBTN.addEventListener("mousedown", function(e) {
		if (taskNum > 0) {
			app.base.changeModel(taskNum, taskNum - 1);
			taskNum--;
		}
	});

	nextTaskBTN.addEventListener("mousedown", function(e) {
		if (taskNum < 5) {
			app.base.changeModel(taskNum, taskNum + 1);
			taskNum++;
		}
	});

	headUpBTN.addEventListener("mousedown", function() {
		app.head.tilt(0.2);
	});

	headDownBTN.addEventListener("mousedown", function() {
		app.head.tilt(-0.2);
	});

    cloudBTN.addEventListener("mousedown", function() {
        if (cloudBTN.innerHTML == "Freeze") {
            cloudBTN.innerHTML = "Unfreeze";
            app.cloudFreezer.freezeCloud();
        } else {
            cloudBTN.innerHTML = "Freeze";
            app.cloudFreezer.unfreezeCloud();
        }
    })

    // for continuous motion: mouse up
    // var btns = [upBTN, downBTN, leftBTN, rightBTN, rotateRightBTN, rotateLeftBTN];
    // for (var i = 0; i < btns.length; i++) {
    // 	btns[i].addEventListener("mouseup", function () {
    //     	clearInterval(this.baseCommand);
    // 	});
    // }

    for (var i = 0; i < camNum; i++) {
        cameras[i].addEventListener("mouseenter", showButtons);
        cameras[i].addEventListener("mouseleave", hideButtons);
    }

    function showButtons() {
    	showButtonsHelper(parseInt(this.id[6]));
    }

    function showButtonsHelper(cam_id) {
    	active_cam = "camera" + cam_id;
        $("#cam" + cam_id).css("color", "yellow");
        layoutButtons(cam_id, btnTop, upBTN, downBTN, leftBTN, rotateRightBTN, rightBTN, rotateLeftBTN); 
    }

    function hideButtons() {
    	hideButtonsHelper(parseInt(this.id[6]));
    }

    function hideButtonsHelper(cam_id) {
    	$("#title" + cam_id).css("color", "blue");
        var camera = document.getElementById("camera" + cam_id);
        camera.removeChild(upBTN);
        camera.removeChild(downBTN);
        camera.removeChild(leftBTN);
        camera.removeChild(rotateRightBTN);
        camera.removeChild(rightBTN);
        camera.removeChild(rotateLeftBTN);
    }

    function layoutButtons(cameraId, btnTop, upBTN, downBTN, leftBTN, rotateRightBTN, rightBTN, rotateLeftBTN) {
    	var camera = document.getElementById("camera" + cameraId);
        camera.appendChild(upBTN);
        camera.appendChild(downBTN);
        camera.appendChild(leftBTN);
        camera.appendChild(rotateRightBTN);
        camera.appendChild(rightBTN);
        camera.appendChild(rotateLeftBTN);

	    var btnTopH = btnTop.offsetHeight;
	    var btnUpH = camera.offsetHeight;// + titleWebH;// + btnTopH;
	    var offset = camera.offsetWidth * cameraId;
	    var divisor = 2 * cameraId;

        var cameraWidth = parseInt(camera.style.width);
        var cameraHeight = parseInt(camera.style.height) - btnTopH;
        var horizontalOffset = HORIZONTAL_BTN_W / 2;
        var verticalOffset = VERTICAL_BTN_W / 2;
	    upBTN.style.left = cameraWidth / 2 - horizontalOffset + "px";
        upBTN.style.top = btnTopH;

    	downBTN.style.left = cameraWidth / 2 - horizontalOffset + "px";
    	downBTN.style.top = cameraHeight + "px";

    	leftBTN.style.left = 0;
    	leftBTN.style.top = cameraHeight / 2 + "px";

    	rotateRightBTN.style.left = VERTICAL_BTN_W + "px";
	    rotateRightBTN.style.top = cameraHeight / 2 + "px";

	    rightBTN.style.left = cameraWidth - VERTICAL_BTN_W + "px";
	    rightBTN.style.top = cameraHeight / 2 + "px";

	    rotateLeftBTN.style.left = cameraWidth - VERTICAL_BTN_W - VERTICAL_BTN_W_WIDE + "px";
	    rotateLeftBTN.style.top = cameraHeight / 2 + "px";
    }

    init_flag = false;

}


function moveBase() {
	app.base.move("left");
}

function stopBase() {
	app.base.endBaseCommand();
}

