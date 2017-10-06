
$(document).ready(function(){

        var elFirstSt =0;
        var elTwoSt =0;
        var elThreeSt =0;

        var cam1Container = document.getElementById("cam1_container");
        var cam2Container = document.getElementById("cam2_container");
        var cam3Container = document.getElementById("cam3_container");


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


    document.getElementById("camera1").style.height = this.cameraHeight + 'px';
    document.getElementById("camera1").style.width = this.cameraWidth + 'px';

    document.getElementById("camera2").style.height = this.cameraHeight + 'px';
    document.getElementById("camera2").style.width = this.cameraWidth + 'px';


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

/*
    var topBTN = document.getElementById("topBTN");
    var sideBTN = document.getElementById("sideBTN");
    var headBTN = document.getElementById("headBTN");


    topBTN.addEventListener("click", getPanel);
    sideBTN.addEventListener("click", getPanel);
    headBTN.addEventListener("click", getPanel);
function getPanel() {
    var btnSel;
    var expr = this.id;
    switch (expr) {
      case 'topBTN':
        btnSel="topBTN";
        break;
      case 'sideBTN':
        btnSel="sideBTN";
        break;
      case 'headBTN':
        btnSel="headBTN";
        break;
    }

    document.getElementById("cmdReceived").innerHTML = btnSel;
}

*/
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



    });