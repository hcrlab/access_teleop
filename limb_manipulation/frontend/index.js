"use strict";

$(function() {
    let self = this;
    let TRAJ_EDIT_SIZE = "2";  // one button click represents 2cm change

    // Cameras
    let CAMERA_NAMES = ["TOP", "LEFT", "FRONT", "HEAD CAMERA"];
    let CAMERA_TOPICS = ['/rviz1/camera1/image', '/rviz1/camera2/image', '/rviz1/camera3/image', '/head_camera/rgb/image_raw'];
    let cameraLargeW = "512px";
    let cameraLargeH = "384px";
    let cameraSmallW = "115px";
    let cameraSmallH = "100px";
    
    // Variables representing program states
    let selectedId = "";  // id of the currently selected body part
    let selectedGrasp = "";  // currently selected grasp type
    let selectedAction = "";  // ABBR of the currently selected action
    let running = false;  // is the program running? (i.e. is "RUN" button clicked?)
    let prevSelectedWaypoint = null;  // the previously selected waypoint during trajectory editing
    let currentStep = -1;  // the current number of step in the current action
    let totalStep = -1;  // the total number of steps in the current action

    // Data structures for storing info received from backend (note: everything is string)
    let parts = new Map();  // body part full name ---> body part id
    let actions = new Map();  // body part id ---> action full name
    let actionsAbbr = new Map();  // action full name ---> action ABBR
    let previewTraj = new Array();  // array of colors of waypoints on the trajectory being previewed

    $(document).ready(function() {
        // ROS setup
        self.ros = new ROSLIB.Ros({
            url : 'ws://localhost:9090'
        });
        self.ros.on('error', function(error) {
            document.getElementById("status_bar").innerHTML = 'Error connecting to websocket server.';
            console.log('Error connecting to websocket server.');
        });
        self.ros.on('connection', function () {
            document.getElementById("status_bar").innerHTML = "We are connected!";
            console.log("We are connected!");
        });
        self.ros.on('close', function(error) {
            document.getElementById("status_bar").innerHTML = 'We lost connection with ROS. All is lost';
            console.error('We lost connection with ROS.');
        });
        // ros topic
        self.serverRequest = new ROSLIB.Topic({
            ros: self.ros,
            name: 'web_app_request',
            messageType: 'limb_manipulation_msgs/WebAppRequest'
        });
        self.serverResponse = new ROSLIB.Topic({
            ros: self.ros,
            name: 'web_app_response',
            messageType: 'limb_manipulation_msgs/WebAppResponse'
        });
        self.serverResponse.subscribe(handleServerStatusResponse);

        // Add camera views
        for (let i = 0; i < CAMERA_NAMES.length; i++) {
            let cameraDiv = document.createElement("div");
            cameraDiv.className = "camera_container";

            let title = document.createElement("p");
            title.innerHTML = CAMERA_NAMES[i];
            let camera = document.createElement("div");
            camera.id = "camera_view_div" + i;
            camera.className = "camera_view";

            cameraDiv.appendChild(title);
            cameraDiv.appendChild(camera);

            if (i == 0) {
                $("#camera_large").append(cameraDiv);
            } else {
                $("#camera_small").append(cameraDiv);
            }

            // add the video stream
            let cameraViewer = new MJPEGCANVAS.Viewer({
                divID : camera.id,
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                // host : 'localhost:8080',
                host : 'localhost',
                width : 640,
                height : 480,
                topic : CAMERA_TOPICS[i]
            });
            camera.firstChild.style.width = i == 0 ? cameraLargeW : cameraSmallW;
            camera.firstChild.style.height = i == 0 ? cameraLargeH : cameraSmallH;
        }
        
        // Initial button states
        $("#action_panel_btn").css({"background-color": "#abaee1", "font-weight": "600"});
        $("#record_panel_btn").css({"background-color": "#d1d4fb", "font-weight": "400"});

        // Event handlers
        // buttons
        document.getElementById("gripper_btn").addEventListener("click", addOrRemoveSakeGripper);
        document.getElementById("record_btn").addEventListener("click", recordScene);
        document.getElementById("yes_octo").addEventListener("click", recordSceneConfirmed);
        document.getElementById("no_octo").addEventListener("click", recordSceneConfirmed);
        document.getElementById("sake_gripper_btn").addEventListener("click", releaseSakeGripper);
        document.getElementById("base_btn").addEventListener("click", showBaseControl);

        document.getElementById("action_panel_btn").addEventListener("click", showPanel);
        document.getElementById("go_traj_btn").addEventListener("click", showGrasp);
        document.getElementById("run_btn").addEventListener("click", run);
        document.getElementById("estop_btn").addEventListener("click", estop);
        
        document.getElementById("ready_for_step_btn").addEventListener("click", getReadyForRun);
        document.getElementById("prev_step_btn").addEventListener("click", gotoPrevStep);
        document.getElementById("next_step_btn").addEventListener("click", gotoNextStep);

        document.getElementById("edit_traj_btn").addEventListener("click", showTraj);
        document.getElementById("cancel_edit").addEventListener("click", saveTraj);
        document.getElementById("save_edit").addEventListener("click", saveTraj);

        document.getElementById("record_panel_btn").addEventListener("click", showPanel);
        document.getElementById("record_action_btn").addEventListener("click", recordAction);

        let graspDropdownLists = document.querySelectorAll(".grasp_dropdown_content a");
        for (let i = 0; i < graspDropdownLists.length; i++) {
            graspDropdownLists[i].addEventListener("click", makeGraspSelection);
        }

        let subGoBtns = document.querySelectorAll(".sub_go_btn");
        for (let i = 0; i < subGoBtns.length; i++) {
            subGoBtns[i].addEventListener("click", doSubAction);
        }

        let arrowBtns = document.querySelectorAll(".arrow");
        for (let i = 0; i < arrowBtns.length; i++) {
            arrowBtns[i].addEventListener("click", recordWaypointChange);
        }
        
        // Camera views
        let cameraViews = document.querySelectorAll(".camera_container");
        for (let i = 0; i < cameraViews.length; i++) {
            cameraViews[i].addEventListener("click", switchCamera);
        }
    });

    function showPanel() {
        // switch between action panel and record panel
        if (this.innerHTML.substring(0, 6) == "Action") {
            $("#action_panel_btn").css({"background-color": "#abaee1", "font-weight": "600"});
            $("#record_panel_btn").css({"background-color": "#d1d4fb", "font-weight": "400"});
            $("#action_step_container").css("display", "grid");
            $("#record_step_container").css("display", "none");
        } else {
            $("#action_panel_btn").css({"background-color": "#d1d4fb", "font-weight": "400"});
            $("#record_panel_btn").css({"background-color": "#abaee1", "font-weight": "600"});
            $("#action_step_container").css("display", "none");
            $("#record_step_container").css("display", "grid");
        }
    }

    function switchCamera() {
        // minimize the large view and maximize the selected view
        let selectedView = this;
        let largeView = document.getElementById("camera_large").querySelector(".camera_container");
        if (largeView != selectedView) {
            // switch DOM element
            document.getElementById("camera_small").replaceChild(largeView, selectedView);
            $("#camera_large").append(selectedView);
            // adjust camera view size
            let largeCameraStyle = largeView.querySelector(".camera_view").firstChild.style;
            largeCameraStyle.width = cameraSmallW;
            largeCameraStyle.height = cameraSmallH;
            let smallCameraStyle = selectedView.querySelector(".camera_view").firstChild.style;
            smallCameraStyle.width = cameraLargeW;
            smallCameraStyle.height = cameraLargeH;
        }
        // hide arrows if the large camera view is the head camera
        if (selectedView.firstChild.innerHTML == "HEAD CAMERA") {
            $(".arrow").css("display", "none");
        } else if (prevSelectedWaypoint != null) {
            $(".arrow").css("display", "block");
        }
    }

    function addOrRemoveSakeGripper() {
        publishRosMsg(this.innerHTML.substring(0, 6).toLowerCase(), []);
        if (this.innerHTML == "Attach Gripper") {
            this.innerHTML = "Remove Gripper";
        } else {
            this.innerHTML = "Attach Gripper";
        }
    }

    function releaseSakeGripper() {
        publishRosMsg("release", []);
    }

    function recordScene() {
        // display the pop up window
        $("#record_btn_confirm").css("display", "block");
        // disable everything in the background
        $("#disable_div").css("display", "block");
    }

    function recordSceneConfirmed() {
        hideControls(3);
        // close the pop up window
        $("#record_btn_confirm").css("display", "none");
        // enable everything in the background
        $("#disable_div").css("display", "none");
        // publish ros message
        let recordOcto = this.innerHTML == "Yes" ? "True" : "";
        publishRosMsg("record", [recordOcto]);
    }

    function recordAction() {
        if (this.innerHTML == "Record") {
            this.innerHTML = "Stop";
        } else {
            this.innerHTML = "Record";
            // TODO: ask to save action or not
        }
    }

    function showBaseControl() {
        let baseCtrlPanel = document.getElementById("base_ctrl");
        if (baseCtrlPanel.style.display == "none") {
            baseCtrlPanel.style.display = "block";
        } else {
            baseCtrlPanel.style.display = "none";
        }
    }

    function makeBodyPartSelection() {
        // show action dropdown list
        $("#action_container").css("display", "block");
        // mark the selection
        let itemName = markDropdownSelection(this);
        // get the item id
        if (parts.has(itemName.toLowerCase())) {
            // a body part is selected
            selectedId = parts.get(itemName.toLowerCase());
            // mark the selection in video stream
            publishRosMsg("prev_id", [selectedId]);
        } else {
            selectedId = "";
            hideControls(3);
        }
        // update available actions for each action dropdown list
        let actionDropdownLists = document.querySelectorAll(".action_dropdown_content");
        let availableActions = actions.get(selectedId);
        for (let i = 0; i < actionDropdownLists.length; i++) {
            // clear previous entiries
            actionDropdownLists[i].innerHTML = "";
            if (selectedId != "") {
                for (let j = -1; j < availableActions.length; j++) {
                    // add DOM element
                    let entry = document.createElement("a");
                    entry.href = "#";
                    if (j == -1) {  // the first entry is always "Select an action"
                        entry.innerHTML = "Select an action";
                    } else {
                        let entryRaw = availableActions[j];
                        entry.innerHTML = entryRaw.length > 1 ? entryRaw.charAt(0).toUpperCase() + entryRaw.slice(1) : entryRaw;
                    }
                    actionDropdownLists[i].appendChild(entry);
                    // add event listener
                    entry.addEventListener("click", makeActionSelection);
                }
            }
        }
    }

    function makeActionSelection() {
        // mark the selection
        let actionName = markDropdownSelection(this).toLowerCase();
        // preview the action in video stream
        if (actionsAbbr.has(actionName)) {
            selectedAction = actionsAbbr.get(actionName);
            publishRosMsg("prev", [selectedAction, selectedId]);
        } else {
            selectedAction = "";
            hideControls(2);
        }
    }

    function showTraj() {
        if (selectedAction != "" && selectedId != "") {
            // preview the trajectory
            publishRosMsg("prev", [selectedAction, selectedId]);
            // display the popup window
            $("#edit_traj_popup").css("display", "block");
            // clear previous waypoints
            $("#waypoints").empty();
            // add waypoints button to the popup window
            for (let i = 0; i < previewTraj.length; i++) {
                let point = document.createElement("button");
                point.className = "point waypoints_btn";
                point.innerHTML = i;
                point.style.backgroundColor = previewTraj[i];
                point.addEventListener("click", highlightWaypoint);

                $("#waypoints").append(point);
            }
            // disable everything in the background
            $(':button').not(".waypoints_btn").prop('disabled', true);
            // left camera view enabled
            $("#camera_large #camera_small").css("z-index", "3");
        }
    }

    function saveTraj() {
        prevSelectedWaypoint = null;
        // close the popup window
        $("#edit_traj_popup").css("display", "none");
        // hide arrows
        $(".arrow").css("display", "none");
        if (this.innerHTML == "Save") {
            // save the trajectory
            publishRosMsg("save_edit", []);
        } else {
            // unsave the trajectory
            publishRosMsg("cancel_edit", []);
        }
        // enable buttons in the background
        $(':button').prop('disabled', false);
        // reset camera view
        $("#cameras").css("z-index", "0");
    }

    function highlightWaypoint() {
        let waypointId = this.innerHTML;
        // highlight the selected waypoint in camera
        publishRosMsg("highlight", [waypointId]);
        if (waypointId == "0") {
            // starting point selected, the user cannot edit the starting point
            alert("Cannot edit the starting point!");
            // highlight the selected button
            this.style.backgroundColor = "#008fb3";
        } else {
            // highlight the selected button
            this.style.backgroundColor = "#00CCFF";
            // clear the previous highlight
            if (prevSelectedWaypoint != null) {
                prevSelectedWaypoint.style.backgroundColor = previewTraj[parseInt(prevSelectedWaypoint.innerHTML)];
            } else {
                document.querySelector("#waypoints button").style.backgroundColor = previewTraj[0];
            }
            // record the selected waypoint
            prevSelectedWaypoint = this;
            // show arrows
            if (document.querySelector("#camera_large p").innerHTML != "HEAD CAMERA") {
                $(".arrow").css("display", "block");
            }            
        }
    }

    function recordWaypointChange() {
        // get the button type
        let type = this.id.split("_")[0];
        // get the camera name
        let cameraTitle = document.querySelector("#camera_large p").innerHTML;
        let camera = "camera1";
        if (cameraTitle == "LEFT") {
            camera = "camera2";
        } else if (cameraTitle == "FRONT") {
            camera = "camera3";
        }
        // highlight the selected waypoint with the change applied
        let deltaX = "0";
        let deltaY = "0";
        if (type == "up") {
            if (cameraTitle == "TOP") {
                deltaX = "-" + TRAJ_EDIT_SIZE;
            } else {
                deltaY = "-" + TRAJ_EDIT_SIZE;
            }
        } else if (type == "down") {
            if (cameraTitle == "TOP") {
                deltaX = TRAJ_EDIT_SIZE;
            } else {
                deltaY = TRAJ_EDIT_SIZE;
            }
        } else if (type == "left") {
            if (cameraTitle == "TOP") {
                deltaY = TRAJ_EDIT_SIZE;
            } else {
                deltaX = "-" + TRAJ_EDIT_SIZE;
            }
        } else {  //right
            if (cameraTitle == "TOP") {
                deltaY = "-" + TRAJ_EDIT_SIZE;
            } else {
                deltaX = TRAJ_EDIT_SIZE;
            }
        }
        publishRosMsg("edit", [prevSelectedWaypoint.innerHTML, deltaX, deltaY, camera]);
    }

    function showGrasp() {
        if (selectedAction != "" && selectedId != "") {
            // show grasp selection dropdown list
            $("#grasp_container_0").css("display", "block");
        }
    }

    function makeGraspSelection() {
        // show run buttons
        $("#run_container").css("display", "block");
        // mark the selection
        let graspType = markDropdownSelection(this).substring(0, 4);
        // record selected grasp type
        if (graspType == "Soft" || graspType == "Hard") {
            selectedGrasp = graspType[0].toLowerCase();
        } else {
            selectedGrasp = "";
            hideControls(1);
        }
    }

    function getReadyForRun() {
        if (previewTraj.length > 0) {  // goto and grasp
            publishRosMsg("step", ["-1"]);
            running = true;
        }
    }

    function gotoPrevStep() {
        currentStep--;
        if (currentStep >= 0) {  // go to previous step
            publishRosMsg("step", [currentStep.toString()]);
        }
    }

    function gotoNextStep() {
        currentStep++;
        if (currentStep < totalStep) {  // go to next step
            publishRosMsg("step", [currentStep.toString()]);
        }
    }

    function run() {
        if (this.innerHTML == "RUN") {
            this.innerHTML == "PAUSE";
            // run all the steps at once
            if (selectedId != "" && selectedGrasp != "" && selectedAction != "") {
                publishRosMsg("run", [selectedId, selectedGrasp, selectedAction]);
                running = true;
            } else {
                document.getElementById("status_bar").innerHTML = "Please make your selections for each step";
            }
        } else {
            this.innerHTML == "RUN";
            // pause //////////////////////////////////////////////////////////////////////////
        }
    }

    function estop() {
        if (this.innerHTML == "STOP") {
            this.innerHTML = "RESET";
            publishRosMsg("stop", []);
        } else {
            this.innerHTML = "STOP";
            publishRosMsg("reset", []);
        }
    }

    function doSubAction() {
        ////////////// UNUSED ///////////////////////////////////////////////////
        let stepAction = this.parentElement.id.split("_")[0];
        // publish ros message associated with the action type
        if (stepAction == "go" && selectedId != "") {
            publishRosMsg("go", [selectedId]);
        } else if (stepAction == "grasp" && selectedId != "" && selectedGrasp != "") {
            publishRosMsg("grasp", [selectedGrasp]);
        } else if (stepAction == "action" && selectedId != "" && selectedGrasp != "" && selectedAction != "") {
            publishRosMsg("do", [selectedAction]);
        } else if (stepAction == "relax" || stepAction == "freeze") {
            publishRosMsg(stepAction, []);
        } else {  // unknown action
            console.log(stepAction);
        }
    }

    function handleServerStatusResponse(msg) {
        if ((msg.type == "run" && msg.msg == "DONE") || msg.type == "prev") {
            // the program has finished running
            running = false;
        }
        if (msg.status && !running) {
            // enable all the buttons
            $(':button').prop('disabled', false);
        }
        if (msg.msg == "") {
            // add something fun
            msg.msg = "^0^";
        }
        // show status
        document.getElementById("status_bar").innerHTML = msg.msg;

        // process returned data
        if (msg.type == "parts" && msg.args.length > 0) {
            // clear previous data
            parts.clear();
            // add the list contents to dropdown menu
            let recordDropdownLists = document.querySelectorAll(".go_dropdown_content");
            for (let i = 0; i < recordDropdownLists.length; i++) {
                recordDropdownLists[i].innerHTML = "";
                for (let j = -1; j < msg.args.length; j++) {
                    // add DOM element
                    let entry = document.createElement("a");
                    entry.href = "#";
                    if (j == -1) {
                        entry.innerHTML = "Select a body part";
                    } else {
                        let msgArgs = msg.args[j].split(":");
                        // update body part information
                        if (i == 0) {
                            parts.set(msgArgs[1], msgArgs[0]);
                        }
                        let entryRaw = msgArgs[1];
                        entry.innerHTML = entryRaw.length > 1 ? entryRaw.charAt(0).toUpperCase() + entryRaw.slice(1) : entryRaw;
                    }
                    recordDropdownLists[i].appendChild(entry);
                    // add event listener
                    entry.addEventListener("click", makeBodyPartSelection);
                }
            }
        } else if (msg.type == "actions" && msg.args.length > 0) {
            // clear previous data
            actions.clear();
            actionsAbbr.clear();
            // update action information
            for (let i = 0; i < msg.args.length; i++) {
                let msgArgs = msg.args[i].split(":");
                // update body part information
                if (!actions.has(msgArgs[0])) {
                    actions.set(msgArgs[0], []);
                }
                let value = actions.get(msgArgs[0]);
                value.push(msgArgs[1]);
                actions.set(msgArgs[0], value);
                actionsAbbr.set(msgArgs[1], msgArgs[2]);
            }
        } else if (msg.type == "prev" && msg.args.length > 0) {
            // clear previous data
            previewTraj.length = 0;
            previewTraj = msg.args;
        } else if (msg.type == "step" && msg.args.length == 1) {
            let stepId = parseInt(msg.args[0]);
            // enable/disable step buttons
            if (stepId == -1) {
                document.getElementById("ready_for_step_btn").disabled = true;
                document.getElementById("prev_step_btn").disabled = false;
                document.getElementById("next_step_btn").disabled = false;
                totalStep = previewTraj.length;
                currentStep = -1;
            }
            if (stepId == totalStep - 2) {
                document.getElementById("next_step_btn").disabled = false;
            }
            if (stepId == 0) {
                document.getElementById("prev_step_btn").disabled = true;
            }
            if (stepId == 1) {
                document.getElementById("prev_step_btn").disabled = false;
            }
            if (stepId == totalStep - 1) {
                document.getElementById("ready_for_step_btn").disabled = false;
                document.getElementById("prev_step_btn").disabled = true;
                document.getElementById("next_step_btn").disabled = true;
                totalStep = -1;
                currentStep = -1;
            }
        }
    }

    /////////////////// Helper functions ///////////////////
    function publishRosMsg(type, args) {
        // disable all the buttons!!!
        $(':button:not(#estop_btn)').prop('disabled', true);
        // publish msg
        let msg = new ROSLIB.Message({
            type: type,
            args: args
        });
        self.serverRequest.publish(msg);
    }

    function markDropdownSelection(selectedItem) {
        // mark the selection and return the selected item name
        selectedItem.parentElement.parentElement.querySelector(".dropbtn").innerHTML = selectedItem.innerHTML;
        return selectedItem.innerHTML;
    }

    function hideControls(num) {
        // hide the main controls
        $("#run_container").css("display", "none");
        if (num == 2) {
            $("#grasp_container_0").css("display", "none");
        }
        if (num == 3) {
            $("#action_container").css("display", "none");
        }
    }

});
