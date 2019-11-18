"use strict";

$(function() {
    let self = this;

    // Constants
    let TRAJ_EDIT_SIZE = "2";  // in trajectory editing: one button click represents 2cm change

    // Cameras
    let CAMERA_NAMES = ["HEAD CAMERA", "TOP", "LEFT", "FRONT"];
    let CAMERA_TOPICS = ["/head_camera/rgb/image_raw", "/rviz1/camera1/image", "/rviz1/camera2/image", "/rviz1/camera3/image"];
    let cameraLargeW = "512px";
    let cameraLargeH = "384px";
    let cameraSmallW = "115px";
    let cameraSmallH = "100px";
    
    // Variables representing program states
    let selectedId = "";  // id of the currently selected body part
    let selectedGrasp = "";  // currently selected grasp type
    let selectedAction = "";  // ABBR of the currently selected action
    let isPreview = false;  // is the program in preview/execution mode?
    let running = false;  // is the program running? (i.e. is "RUN" button clicked?)

    let prevSelectedWaypoint = null;  // the previously selected waypoint during trajectory editing
    let currentStep = -1;  // the current number of step in the current action
    let totalStep = -1;  // the total number of steps in the current action

    // Data structures for storing info received from backend (note: everything is string)
    let parts = new Map();  // body part full name (all lowercase) ---> body part id
    let actions = new Map();  // body part id ---> action full name (all lowercase)
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
        // ROS topic
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

            if (i === 0) {
                $("#camera_large").append(cameraDiv);
            } else {
                $("#camera_small").append(cameraDiv);
            }

            // add the video stream
            let cameraViewer = new MJPEGCANVAS.Viewer({
                divID : camera.id,
                host : 'localhost:8080',
                width : 640,
                height : 480,
                topic : CAMERA_TOPICS[i]
            });
            camera.firstChild.style.width = i === 0 ? cameraLargeW : cameraSmallW;
            camera.firstChild.style.height = i === 0 ? cameraLargeH : cameraSmallH;
        }

        // Event handlers
        // buttons
        $("#gripper_btn").click(addOrRemoveSakeGripper);
        $("#record_btn").click(recordScene);
        $("#yes_octo").click(recordSceneConfirmed);
        $("#no_octo").click(recordSceneConfirmed);
        $("#base_btn").click(showBaseControl);
        $("#sake_gripper_btn").click(openSakeGripper);
        $("#reset_btn").click(resetArm);
        $("#help_btn").click(showHelpInfo);

        $("#preview_checkbox").click(showPreview);

        $("#edit_traj_btn").click(showTraj);
        $("#cancel_edit").click(saveTraj);
        $("#save_edit").click(saveTraj);
        $("#ready_for_step_btn").click(getReadyForRun);
        $("#prev_step_btn").click(gotoPrevStep);
        $("#next_step_btn").click(gotoNextStep);
        $("#run_btn").click(run);
        $("#estop_btn").click(estop);


        //////////////////////////////////////////////////////
        // NEW ///////////////////////////
        $("#back_to_selection").click(backToSelection);

        let bodySelectionBtns = document.querySelectorAll(".body_selection_btn");
        for (let i = 0; i < bodySelectionBtns.length; i++) {
            bodySelectionBtns[i].addEventListener("click", makeBodyPartSelection);
        }
        // END NEW ///////////////////////////////////////

        let popupCloseBtns = document.querySelectorAll(".popup_close_btn");
        for (let i = 0; i < popupCloseBtns.length; i++) {
            popupCloseBtns[i].addEventListener("click", closePopup);
        }

        let graspDropdownLists = document.querySelectorAll(".grasp_dropdown_content a");
        for (let i = 0; i < graspDropdownLists.length; i++) {
            graspDropdownLists[i].addEventListener("click", makeGraspSelection);
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

    function showPreview() {
        // switch between preview mode and execution mode
        let checkmark = document.getElementById("checkmark");
        if (checkmark.style.display === "none") {
            // change to preview mode
            isPreview = true;
            checkmark.style.display = "inline-block";
            // hide grasp type selection dropdown menu
            $("#grasp_container").css("display", "none");
        } else {
            // change to execution mode
            isPreview = false;
            checkmark.style.display = "none";
            // show grasp type selection dropdown menu
            $("#grasp_container").css("display", "block");
        }
        // initialize the states
        // disable prev step and next step buttons
        document.getElementById("prev_step_btn").disabled = true;
        document.getElementById("next_step_btn").disabled = true;
        ////////////////////////////////////////////////////////////////////////////////////
        // move the robot arm to the origianl position


        // reset all the state variables
        // let currentStep = -1;  // the current number of step in the current action
        // let totalStep = -1;  // the total number of steps in the current action
        ////////////////////////////////////////////////////////////////////////////////////
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
        if (selectedView.firstChild.innerHTML === "HEAD CAMERA") {
            $(".arrow").css("display", "none");
        } else if (prevSelectedWaypoint != null) {
            $(".arrow").css("display", "block");
        }
    }

    function addOrRemoveSakeGripper() {
        publishRosMsg(this.innerHTML.substring(0, 6).toLowerCase(), []);
        if (this.innerHTML === "Attach Gripper") {
            this.innerHTML = "Remove Gripper";
        } else {
            this.innerHTML = "Attach Gripper";
        }
    }

    function recordScene() {
        // display the pop up window
        $("#record_btn_confirm").css("display", "block");
        // disable everything in the background
        $("#disable_div").css("display", "block");
    }

    function recordSceneConfirmed() {
        //////////////////////////////////////////////////////////////////////////
        // OLD
        // // hide all the controls
        // hideControls(3);
        /////////////////////////////////////////////////////////////////////////

        // close the pop up window
        $("#record_btn_confirm").css("display", "none");
        // enable everything in the background
        $("#disable_div").css("display", "none");
        // publish ros message
        let recordOcto = this.innerHTML === "Yes" ? "True" : "";
        publishRosMsg("record", [recordOcto]);
    }

    function showBaseControl() {
        let baseCtrlPanel = document.getElementById("base_ctrl");
        if (baseCtrlPanel.style.display === "none") {
            baseCtrlPanel.style.display = "block";
        } else {
            baseCtrlPanel.style.display = "none";
        }
    }

    function openSakeGripper() {
        publishRosMsg("open", []);
    }

    function resetArm() {
        publishRosMsg("reset", []);
    }

    function showHelpInfo() {
        // display the pop up window
        $("#help_popup").css("display", "block");
        // disable everything in the background
        $("#disable_div").css("display", "block");
    }

    function closePopup() {
        // close the pop up window
        this.parentElement.style.display = "none";
        // enable everything in the background
        $("#disable_div").css("display", "none");
    }

    function makeBodyPartSelection() {        
        // OLD ////////////////////////////////////////////////////////////////////
        // // show action dropdown list
        // $("#action_container").css("display", "block");
        // // hide trajectory editing buttons
        // $("#edit_traj_btn").css("display", "none");
        // $("#go_traj_btn").css("display", "none");
        // // mark the selection
        // let itemName = markDropdownSelection(this);
        // // get the item id
        // if (parts.has(itemName.toLowerCase())) {  // a body part is selected
        //     selectedId = parts.get(itemName.toLowerCase());
        //     // mark the selection in video stream
        //     publishRosMsg("prev_id", [selectedId]);
        // } else {  // nothing selected
        //     selectedId = "";
        //     hideControls(3);
        // }
        // // update available actions for each action dropdown list
        // let actionDropdownLists = document.querySelectorAll(".action_dropdown_content");
        // let availableActions = actions.get(selectedId);
        // for (let i = 0; i < actionDropdownLists.length; i++) {
        //     // clear previous entiries
        //     document.getElementById("action_btn").innerHTML = "Select an action";
        //     actionDropdownLists[i].innerHTML = "";
        //     if (selectedId != "") {
        //         for (let j = -1; j < availableActions.length; j++) {
        //             // add DOM element
        //             let entry = document.createElement("a");
        //             entry.href = "#";
        //             if (j === -1) {  // the first entry is always "Select an action"
        //                 entry.innerHTML = "Select an action";
        //             } else {
        //                 let entryRaw = availableActions[j];
        //                 entry.innerHTML = capitalize(entryRaw);
        //             }
        //             actionDropdownLists[i].appendChild(entry);
        //             // add event listener
        //             entry.addEventListener("click", makeActionSelection);
        //         }
        //     }
        // }
        // END OLD ////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////
        // NEW //////////////////////////////////////////////////////
        // mark the selection
        let itemName = this.innerHTML.split("<br>").join(" ").toLowerCase();
        // get the item id
        if (parts.has(itemName.toLowerCase())) {  // a body part is selected
            // show action selection popup window
            document.getElementById("body_actions_target").innerHTML += itemName + ":";
            $("#body_actions_popup").css("display", "block");
            // disable everything in the background
            $("#disable_div").css("display", "block");
            selectedId = parts.get(itemName.toLowerCase());
            // mark the selection in video stream
            publishRosMsg("prev_id", [selectedId]);
            // add available actions to the pop up window
            let bodyActionsList = document.getElementById("body_actions_list");
            let availableActions = actions.get(selectedId);
            // clear previous entiries
            bodyActionsList.innerHTML = "";
            for (let i = 0; i < availableActions.length; i++) {
                // add DOM element
                let entry = document.createElement("button");
                entry.className = "body_action_btn";
                let entryRaw = availableActions[i];
                entry.innerHTML = capitalize(entryRaw);
                bodyActionsList.appendChild(entry);
                // add event listener
                entry.addEventListener("click", makeActionSelection);
            }
        } else {  // nothing selected
            selectedId = "";
        }
        // END NEW //////////////////////////////////////////////////////////
    }

    function makeActionSelection() {
        ///////////////////////////////////////////////////////
        // OLD ////////////////////////////////////////////////////////////////////
        // // mark the selection
        // let actionName = markDropdownSelection(this).toLowerCase();
        // if (actionsAbbr.has(actionName)) {  // an action is selected
        //     selectedAction = actionsAbbr.get(actionName);
        //     // preview the action in video stream
        //     publishRosMsg("prev", [selectedAction, selectedId]);
        //     // show trajectory editing buttons
        //     $("#edit_traj_btn").css("display", "inline-block");
        //     $("#go_traj_btn").css("display", "inline-block");
        // } else {  // nothing selected
        //     selectedAction = "";
        //     hideControls(2);
        // }
        // END OLD ///////////////////////////////////////////////

        /////////////////////////////////////////////////
        // NEW /////////////////////////////////////////////////
        // mark the selection
        let actionName = this.innerHTML.toLowerCase();
        if (actionsAbbr.has(actionName)) {  // an action is selected
            // enable everything in the background (remove the disable div)
            $("#disable_div").css("display", "none");
            selectedAction = actionsAbbr.get(actionName);
            // preview the action in video stream
            publishRosMsg("prev", [selectedAction, selectedId]);
            // display the action name
            document.getElementById("current_action_name").innerHTML = capitalize(actionName);
            // show the window for robot controls
            $("#main_window").css("display", "block");
            // hide the window for selection
            $("#selection").css("display", "none");
        } else {  // nothing selected
            selectedAction = "";
        }
        // END NEW //////////////////////////////////////////////
    }

    function backToSelection() {
        // hide the window for robot controls
        $("#main_window").css("display", "none");
        // show the window for selection
        $("#selection").css("display", "block");
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
        if (this.innerHTML === "Save") {  // save the trajectory
            publishRosMsg("save_edit", []);
        } else {  // unsave the trajectory
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
        if (waypointId === "0") {
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
        if (cameraTitle === "LEFT") {
            camera = "camera2";
        } else if (cameraTitle === "FRONT") {
            camera = "camera3";
        }
        // highlight the selected waypoint with the change applied
        let deltaX = "0";
        let deltaY = "0";
        if (type === "up") {
            if (cameraTitle === "TOP") {
                deltaX = "-" + TRAJ_EDIT_SIZE;
            } else {
                deltaY = "-" + TRAJ_EDIT_SIZE;
            }
        } else if (type === "down") {
            if (cameraTitle === "TOP") {
                deltaX = TRAJ_EDIT_SIZE;
            } else {
                deltaY = TRAJ_EDIT_SIZE;
            }
        } else if (type === "left") {
            if (cameraTitle === "TOP") {
                deltaY = TRAJ_EDIT_SIZE;
            } else {
                deltaX = "-" + TRAJ_EDIT_SIZE;
            }
        } else {  // type === right
            if (cameraTitle === "TOP") {
                deltaY = "-" + TRAJ_EDIT_SIZE;
            } else {
                deltaX = TRAJ_EDIT_SIZE;
            }
        }
        publishRosMsg("edit", [prevSelectedWaypoint.innerHTML, deltaX, deltaY, camera]);
    }

    function makeGraspSelection() {
        // show run buttons
        $("#run_container").css("display", "block");
        // mark the selection
        let graspType = markDropdownSelection(this).substring(0, 4);
        if (graspType === "Soft" || graspType === "Hard") {  // record selected grasp type
            selectedGrasp = graspType[0].toLowerCase();
        } else {  // nothing selected
            selectedGrasp = "";
            hideControls(1);
        }
    }

    function getReadyForRun() {
        if (previewTraj.length > 0) {  
            // valid trajectory, goto the body part and grasp
            publishRosMsg("step", ["-1", selectedId, selectedGrasp]);
            running = true;
            currentStep++;
        }
    }

    function gotoPrevStep() {
        if (currentStep - 1 > -1) {
            // not at the first step, go to the previous step
            currentStep--;
            publishRosMsg("step", [currentStep.toString(), selectedGrasp]);
        }
    }

    function gotoNextStep() {
        if (currentStep + 1 < totalStep) {
            // not at the last step, go to the next step
            currentStep++;
            publishRosMsg("step", [currentStep.toString(), selectedGrasp]);
        }
    }

    function run() {
        if (this.innerHTML === "RUN") {
            this.innerHTML = "PAUSE";
            if (!running) {  // run all the steps at once
                if (selectedId != "" && selectedGrasp != "" && selectedAction != "") {
                    publishRosMsg("run", [selectedId, selectedGrasp, selectedAction]);
                    running = true;
                    this.disabled = false;  // enable this button so that the user can pause
                } else {
                    document.getElementById("status_bar").innerHTML = "Please make your selections for each step";
                }                
            } else {  // continue the current action
                publishRosMsg("continue", []);
            }

        } else {  // pause the current action
            this.innerHTML = "RUN";
            publishRosMsg("pause", []);
        }
    }

    function estop() {
        if (this.innerHTML === "STOP") {
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
        if (stepAction === "go" && selectedId != "") {
            publishRosMsg("go", [selectedId]);
        } else if (stepAction === "grasp" && selectedId != "" && selectedGrasp != "") {
            publishRosMsg("grasp", [selectedGrasp]);
        } else if (stepAction === "action" && selectedId != "" && selectedGrasp != "" && selectedAction != "") {
            publishRosMsg("do", [selectedAction]);
        } else if (stepAction === "relax" || stepAction === "freeze") {
            publishRosMsg(stepAction, []);
        } else {  // unknown action
            console.log(stepAction);
        }
    }

    function handleServerStatusResponse(msg) {
        if ((msg.type === "run" && msg.msg === "DONE") || msg.type === "step") {
            // the program has finished running
            running = false;
            document.getElementById("run_btn").innerHTML = "RUN";
        }
        if (msg.status && !running) {
            // enable all the buttons
            $(':button').prop('disabled', false);
        }
        if (msg.msg === "") {  // add something fun
            msg.msg = "^0^";
        }
        // show status
        document.getElementById("status_bar").innerHTML = msg.msg;

        // process returned data
        if (msg.type === "parts" && msg.args.length > 0) {
            // clear previous data
            parts.clear();

            // /////////////////////////////////////////////////////
            // // OLD ///////////////////////////////////
            // // add the list contents to menu
            // let recordDropdownLists = document.querySelectorAll(".go_dropdown_content");
            // for (let i = 0; i < recordDropdownLists.length; i++) {
            //     document.getElementById("go_btn").innerHTML = "Select a body part";
            //     selectedId = "";
            //     recordDropdownLists[i].innerHTML = "";
            //     for (let j = -1; j < msg.args.length; j++) {
            //         // add DOM element
            //         let entry = document.createElement("a");
            //         entry.href = "#";
            //         if (j === -1) {
            //             entry.innerHTML = "Select a body part";
            //         } else {
            //             let msgArgs = msg.args[j].split(":");
            //             // update body part information
            //             if (i === 0) {
            //                 parts.set(msgArgs[1], msgArgs[0]);
            //             }
            //             let entryRaw = msgArgs[1];
            //             entry.innerHTML = capitalize(entryRaw);
            //         }
            //         recordDropdownLists[i].appendChild(entry);
            //         // add event listener
            //         entry.addEventListener("click", makeBodyPartSelection);
            //     }
            // }
            // // END OLD ////////////////////////////////////



            ////////////////////////////////////////////////////
            // NEW ////////////////////////////////////////////////
            // disable all the body part selection buttons
            $('.body_selection_btn').prop('disabled', true);
            selectedId = "";
            for (let i = 0; i < msg.args.length; i++) {
                // update body part information
                let msgArgs = msg.args[i].split(":");
                parts.set(msgArgs[1], msgArgs[0]);
                let entryIdName = msgArgs[1].split(" ").join("_").toLowerCase();
                // enable body selection button corresponding to this msg
                let bodyPartBtn = document.getElementById(entryIdName);
                if (bodyPartBtn != null) {
                    bodyPartBtn.disabled = false;
                }
            }
            // END NEW ///////////////////////////////////////////////////



        } else if (msg.type === "actions" && msg.args.length > 0) {
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
        } else if (msg.type === "prev" && msg.args.length > 0) {
            // clear previous data
            previewTraj.length = 0;
            previewTraj = msg.args;
        } else if (msg.type === "step") {
            let stepId = parseInt(msg.args[0]);
            document.getElementById("ready_for_step_btn").disabled = false;
            document.getElementById("prev_step_btn").disabled = false;
            document.getElementById("next_step_btn").disabled = false;

            // enable/disable step buttons
            if (stepId <= 0) {  // starting point
                document.getElementById("ready_for_step_btn").disabled = true;
                document.getElementById("prev_step_btn").disabled = true;
                document.getElementById("next_step_btn").disabled = false;
                totalStep = previewTraj.length;
                currentStep = 0;
            }
            if (stepId === totalStep - 1) {  // last point
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

    function capitalize(entryRaw) {
        // return the capitalized entry
        return entryRaw.length > 1 ? entryRaw.charAt(0).toUpperCase() + entryRaw.slice(1).toLowerCase() : entryRaw;
    }

});
