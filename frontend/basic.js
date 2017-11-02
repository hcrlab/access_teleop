var elFirstSt =0;
var elTwoSt =0;
var elThreeSt =0;
var winHeight = 0; /* Window height */
var winWidth = 0;  /* Window width */

$(document).ready(function() {
    var cam1Container = document.getElementById("cam1-container");
    var cam2Container = document.getElementById("cam2-container");
    var cam3Container = document.getElementById("cam3-container");

    this.backendCameraWidth = "640";
    this.backendCameraHeight = "480";

    var aspectRatio = this.backendCameraWidth / this.backendCameraHeight;

    var dimCam1Width = cam1Container.clientWidth;
    var dimCam1Height = cam1Container.clientWidth / aspectRatio;

    var dimCam2Width = cam2Container.clientWidth;
    var dimCam2Height = cam2Container.clientWidth / aspectRatio;

    var dimCam3Width = cam3Container.clientWidth;
    var dimCam3Height = cam3Container.clientWidth / aspectRatio;

    var camera1 = document.getElementById("camera1");
    var camera2 = document.getElementById("camera2");
    var camera3 = document.getElementById("camera3");
    var elTitle = document.getElementById("title");
    var elTopBTN = document.getElementById("topBTN");
    var elcmdReceived = document.getElementById("cmdReceived");
    var dimcmdReceivedHeight = elcmdReceived.clientHeight;

    var dimTitleHeight = elTitle.offsetHeight;
    var dimTopBTNHeight = elTopBTN.offsetHeight;

    // Create the main viewer.
    var viewer1 = new MJPEGCANVAS.Viewer({
        divID: 'camera1',
        host: 'localhost',
        width: dimCam1Width,
        height: dimCam1Height,
        topic: '/rviz1/camera1/image'
    });

    // Create the main viewer.
    var viewer2 = new MJPEGCANVAS.Viewer({
        divID: 'camera2',
        host: 'localhost',
        width: dimCam2Width,
        height: dimCam2Height,
        topic: '/rviz1/camera2/image'
    });

    // Create the main viewer.
    var viewer3 = new MJPEGCANVAS.Viewer({
        divID: 'camera3',
        host: 'localhost',
        width: dimCam3Width,
        height: dimCam3Height,
        topic: '/head_camera/rgb/image_raw'
    });

    /**/
    var topBTN = document.getElementById("topBTN");
    var sideBTN = document.getElementById("sideBTN");
    var headBTN = document.getElementById("headBTN");

    $("#cameraTop").on('shown.bs.collapse', shownPanel);
    $("#cameraSide").on('shown.bs.collapse', shownPanel);
    $("#cameraHead").on('shown.bs.collapse', shownPanel);

    $("#cameraTop").on('hidden.bs.collapse', hidePanel);
    $("#cameraSide").on('hidden.bs.collapse', hidePanel);
    $("#cameraHead").on('hidden.bs.collapse', hidePanel);

    function resizeWindow() {

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

            viewer1.canvas.width = dimCam1Width;
            viewer2.canvas.width = dimCam2Width;
            viewer3.canvas.width = dimCam3Width;
            viewer1.width = dimCam1Width;
            viewer2.width = dimCam2Width;
            viewer3.width = dimCam3Width;
            viewer1.canvas.height = viewer1.height = dimCam1Height;
            viewer2.canvas.height = viewer2.height = dimCam2Height;
            viewer3.canvas.height = viewer3.height = dimCam3Height;
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
            /*            cam1Container.style.width = dimCam1Width + 'px';
                        cam2Container.style.width = dimCam2Width + 'px';
                        cam3Container.style.width = dimCam3Width + 'px';*/
        } else {
            dimCam1Width = winWidth / 3;//cam1Container.clientWidth;
            dimCam2Width = winWidth / 3;//cam2Container.clientWidth;
            dimCam3Width = winWidth / 3;//cam3Container.clientWidth;
            dimCam1Height = dimCam1Width / aspectRatio;
            dimCam2Height = dimCam2Width / aspectRatio;
            dimCam3Height = dimCam3Width / aspectRatio;
        }


        viewer1.width = viewer1.canvas.width = dimCam1Width;
        viewer2.width = viewer2.canvas.width = dimCam2Width;
        viewer3.width = viewer3.canvas.width = dimCam3Width;

        viewer1.canvas.height = viewer1.height = dimCam1Height;
        viewer2.canvas.height = viewer2.height = dimCam2Height;
        viewer3.canvas.height = viewer3.height = dimCam3Height;
    }

    $(window).resize(function () {
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
            /*            cam1Container.style.width = dimCam1Width + 'px';
                        cam2Container.style.width = dimCam2Width + 'px';
                        cam3Container.style.width = dimCam3Width + 'px';*/
        } else {
            dimCam1Width = winWidth / 3;//cam1Container.clientWidth;
            dimCam2Width = winWidth / 3;//cam2Container.clientWidth;
            dimCam3Width = winWidth / 3;//cam3Container.clientWidth;
            dimCam1Height = dimCam1Width / aspectRatio;
            dimCam2Height = dimCam2Width / aspectRatio;
            dimCam3Height = dimCam3Width / aspectRatio;
        }


        viewer1.width = viewer1.canvas.width = dimCam1Width;
        viewer2.width = viewer2.canvas.width = dimCam2Width;
        viewer3.width = viewer3.canvas.width = dimCam3Width;

        viewer1.canvas.height = viewer1.height = dimCam1Height;
        viewer2.canvas.height = viewer2.height = dimCam2Height;
        viewer3.canvas.height = viewer3.height = dimCam3Height;
    });

    topBTN.addEventListener("click", getPanel);
    sideBTN.addEventListener("click", getPanel);
    headBTN.addEventListener("click", getPanel);

    function getPanel() {
        var btnSel;
        var expr = this.id;
        switch (expr) {
            case 'topBTN':
                btnSel = "topBTN";
                break;
            case 'sideBTN':
                btnSel = "sideBTN";
                break;
            case 'headBTN':
                btnSel = "headBTN";
                break;
        }

        document.getElementById("cmdReceived").innerHTML = btnSel;
    }

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


});