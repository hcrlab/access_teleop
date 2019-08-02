"use strict";

$(function() {
    let CAMERA_NAMES = ["TOP", "FRONT", "LEFT"];
    let CAMERA_TOPICS = ['/head_camera/rgb/image_raw', '/rviz1/camera1/image', '/rviz1/camera2/image', '/rviz1/camera3/image'];
    let self = this;

    let parts = new Map();  // body part full name ---> body part id
    let actions = new Map();  // body part id ---> [action full name, action abbr]

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
            console.error('We lost connection with ROS. All is lost');
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

        // add camera views
        for (let i = 0; i < 3; i++) {
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
                host : 'localhost',
                width : 480,
                height : 480,
                topic : CAMERA_TOPICS[i]
            });
            let cameraSize = i == 0 ? "350px" : "150px";
            camera.firstChild.style.width = cameraSize;
            camera.firstChild.style.height = cameraSize;
        }
        
        // initial button states
        $("#action_panel_btn").css({"background-color": "#abaee1", "font-weight": "600"});
        $("#record_panel_btn").css({"background-color": "#d1d4fb", "font-weight": "400"});

        // event handlers
        // buttons
        document.getElementById("gripper_btn").addEventListener("click", addOrRemoveSakeGripper);
        document.getElementById("record_btn").addEventListener("click", recordScene);
        document.getElementById("yes_octo").addEventListener("click", recordSceneConfirmed);
        document.getElementById("no_octo").addEventListener("click", recordSceneConfirmed);
        document.getElementById("record_action_btn").addEventListener("click", recordAction);
        document.getElementById("action_panel_btn").addEventListener("click", showPanel);
        document.getElementById("record_panel_btn").addEventListener("click", showPanel);
        let graspDropdownLists = document.querySelectorAll(".grasp_dropdown_content a");
        for (let i = 0; i < graspDropdownLists.length; i++) {
            graspDropdownLists[i].addEventListener("click", makeGraspSelection);
        }
        // camera views
        let cameraViews = document.querySelectorAll(".camera_container");
        for (let i = 0; i < cameraViews.length; i++) {
            cameraViews[i].addEventListener("click", switchCamera);
        }
    });

    function publishRosMsg(type, args) {
        let msg = new ROSLIB.Message({
            type: type,
            args: args
        });
        self.serverRequest.publish(msg);
    };

    function addOrRemoveSakeGripper() {
        publishRosMsg(this.innerHTML.substring(0, 6).toLowerCase(), []);
        if (this.innerHTML == "Attach Gripper") {
            this.innerHTML = "Remove Gripper";
        } else {
            this.innerHTML = "Attach Gripper";
        }
    }

    function recordScene() {
        $("#record_btn_confirm").css("display", "block");
    }

    function recordSceneConfirmed() {
        $("#record_btn_confirm").css("display", "none");
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

    function showPanel() {
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
        // remove the large view
        let selectedView = this.parentElement.querySelector(".camera_container");
        let largeView = document.getElementById("camera_large").querySelector(".camera_container");
        if (largeView != selectedView) {
            let largeSize = largeView.querySelector(".camera_view").firstChild.style.width;
            let smallSize = selectedView.querySelector(".camera_view").firstChild.style.width;
            $("#camera_large").remove(largeView);
            $("#camera_small").append(largeView);
            largeView.querySelector(".camera_view").firstChild.style.width = smallSize;
            largeView.querySelector(".camera_view").firstChild.style.height = smallSize;
            // add the selected view
            $("#camera_large").append(selectedView);
            $("#camera_small").remove(selectedView);
            selectedView.querySelector(".camera_view").firstChild.style.width = largeSize;
            selectedView.querySelector(".camera_view").firstChild.style.height = largeSize;
        }
    }

    function markDropdownSelection(selectedItem) {
        // mark the selection
        selectedItem.parentElement.parentElement.querySelector(".dropbtn").innerHTML = selectedItem.innerHTML;
        return selectedItem.innerHTML;
    }

    function makeBodyPartSelection() {
        // mark the selection
        let itemName = markDropdownSelection(this);
        // update available actions
        let actionDropdownLists = document.querySelectorAll(".action_dropdown_content");
        let availableActions = actions.get(parts.get(itemName.toLowerCase()));
        for (let i = 0; i < actionDropdownLists.length; i++) {
            actionDropdownLists[i].innerHTML = "";
            if (parts.has(itemName.toLowerCase())) {
                for (let j = -1; j < availableActions.length; j++) {
                    // add DOM element
                    let entry = document.createElement("a");
                    entry.href = "#";
                    if (j == -1) {
                        entry.innerHTML = "Select an action";
                    } else {
                        let entryRaw = availableActions[j][0];
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
        markDropdownSelection(this);
    }

    function makeGraspSelection() {
        // mark the selection
        markDropdownSelection(this);
    }

    function handleServerStatusResponse(msg) {
        document.getElementById("status_bar").innerHTML = msg.msg;
        if (msg.type == "parts" && msg.args.length > 0) {
            // add the list contents to dropdown menu
            let recordDropdownLists = document.querySelectorAll(".go_dropdown_content");
            parts.clear();
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
            // update action information
            for (let i = 0; i < msg.args.length; i++) {
                let msgArgs = msg.args[i].split(":");
                // update body part information
                if (!actions.has(msgArgs[0])) {
                    actions.set(msgArgs[0], []);
                }
                let value = actions.get(msgArgs[0]);
                value.push([msgArgs[1], msgArgs[2]]);
                actions.set(msgArgs[0], value);
            }
        }
    }

});
