"use strict";

$(function() {
    var CAMERA_NAMES = ["TOP", "FRONT", "LEFT"];
    var self = this;

    $(document).ready(function() {
        // ROS setup
        self.ros = new ROSLIB.Ros({
            url : 'ws://localhost:9090'
        });
        self.ros.on('error', function(error) {
            console.log('Error connecting to websocket server.');
        });
        // ros topic
        self.serverRequest = new ROSLIB.Topic({
            ros: self.ros,
            name: 'web_app_request',
            messageType: 'limb_manipulation_msgs/WebAppRequest'
        });

        // add camera views
        for (var i = 0; i < 3; i++) {
            var cameraDiv = document.createElement("div");
            cameraDiv.className = "camera_container";

            var title = document.createElement("p");
            title.innerHTML = CAMERA_NAMES[i];
            var camera = document.createElement("img");
            camera.className = "camera_view";
            // TODO: change this into the real url for camera stream
            camera.src = "blue.png";
            // camera.src = CAMERA_NAMES[i].toLowerCase() + ".jpg";
            // END TODO
            camera.alt = CAMERA_NAMES[i] + " camera";
            camera.style.width = i == 0 ? "350px" : "150px";
            camera.style.height = i == 0 ? "350px" : "150px";

            cameraDiv.appendChild(title);
            cameraDiv.appendChild(camera);

            if (i == 0) {
                $("#camera_large").append(cameraDiv);
            } else {
                $("#camera_small").append(cameraDiv);
            }
        }
        
        // initial button states
        $("#action_panel_btn").css("background-color", "#bee0fa");
        $("#record_panel_btn").css("background-color", "#f2f2f2");

        // event handlers
        // buttons
        document.getElementById("gripper_btn").addEventListener("click", addOrRemoveSakeGripper);
        document.getElementById("recrod_btn").addEventListener("click", recordAction);
        document.getElementById("action_panel_btn").addEventListener("click", showPanel);
        document.getElementById("record_panel_btn").addEventListener("click", showPanel);

        var cameraViews = document.querySelectorAll(".camera_container");
        for (var i = 0; i < cameraViews.length; i++) {
            cameraViews[i].addEventListener("click", switchCamera);
        }
    });

    function publishRosMsg(type, args) {
        var msg = new ROSLIB.Message({
            type: type,
            args: args
        });
        self.serverRequest.publish(msg);
    };

    function addOrRemoveSakeGripper() {
        if (this.innerHTML == "Attach Gripper") {
            this.innerHTML = "Remove Gripper";
        } else {
            this.innerHTML = "Attach Gripper";
        }
        publishRosMsg(this.innerHTML.substring(0, 6).toLowerCase(), []);
    }

    function recordAction() {
        if (this.innerHTML == "Recrod") {
            this.innerHTML = "Stop";
        } else {
            this.innerHTML = "Record";
            // TODO: ask to save action or not
        }
    }

    function showPanel() {
        if (this.innerHTML.substring(0, 6) == "Action") {
            $("#action_panel_btn").css("background-color", "#bee0fa");
            $("#record_panel_btn").css("background-color", "#e5e5e5");
            $("#action_step_container").css("display", "grid");
            $("#record_step_container").css("display", "none");
        } else {
            $("#action_panel_btn").css("background-color", "#e5e5e5");
            $("#record_panel_btn").css("background-color", "#bee0fa");
            $("#action_step_container").css("display", "none");
            $("#record_step_container").css("display", "grid");
        }
        
    }

    function switchCamera() {
        // remove the large view
        var selectedView = this.parentElement;
        var largeView = document.getElementById("camera_large").querySelector(".camera_container");
        if (largeView != selectedView) {
            var largeSize = largeView.querySelector(".camera_view").style.width;
            var smallSize = selectedView.querySelector(".camera_view").style.width;
            $("#camera_large").remove(largeView);
            $("#camera_small").append(largeView);
            largeView.querySelector(".camera_view").style.width = smallSize;
            largeView.querySelector(".camera_view").style.height = smallSize;
            // add the selected view
            $("#camera_large").append(selectedView);
            $("#camera_small").remove(selectedView);
            selectedView.querySelector(".camera_view").style.width = largeSize;
            selectedView.querySelector(".camera_view").style.height = largeSize;
        }
    }
});
