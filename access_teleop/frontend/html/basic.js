
$(document).ready(function(){

    var elFirstSt =0;
    var elTwoSt =0;
    var elThreeSt =0;

    var elFirst = document.getElementById("first");
    var elTwo = document.getElementById("two");
    var elThree = document.getElementById("three");


    var dimCam1Width = elFirst.clientWidth;
    var dimCam1Height = elFirst.clientWidth;//dimCam1.clientHeight;

    // Create the main viewer.
    var viewer1 = new MJPEGCANVAS.Viewer({
      divID : 'camera1',
      host : 'localhost',
      width : dimCam1Width,
      height : dimCam1Height,
      topic : '/rviz1/camera1/image'
    });


    var dimCam2Width = elTwo.clientWidth;
    var dimCam2Height = elTwo.clientWidth;//dimCam2.clientHeight;

    // Create the main viewer.
    var viewer2 = new MJPEGCANVAS.Viewer({
      divID : 'camera2',
      host : 'localhost',
      width : dimCam2Width,
      height : dimCam2Height,
      topic : '/rviz1/camera2/image'
    });


    var dimCam3Width = elThree.clientWidth;
    var dimCam3Height = elThree.clientWidth;//dimCam2.clientHeight;

    // Create the main viewer.
    var viewer3= new MJPEGCANVAS.Viewer({
      divID : 'camera3',
      host : 'localhost',
      width : dimCam3Width,
      height : dimCam3Height,
      topic : '/rviz1/camera3/image' // '/head_camera/rgb/image_raw'
    });


    var topBTN = document.getElementById("topBTN");
    var sideBTN = document.getElementById("sideBTN");
    var headBTN = document.getElementById("headBTN");

    viewer2.on("resize",  function(){


    });
    /*
    */
    topBTN.addEventListener("click", getPanel);
    sideBTN.addEventListener("click", getPanel);
    sideBTN.addEventListener("click", getPanel);

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

function hidePanel() {
    var expr = this.id;

    switch (expr){
        case 'cameraTop':
            elFirstSt =1;
            elFirst.classList.add("col-sm-1");
            elFirst.classList.remove("col-sm-10");
            elFirst.classList.remove("col-sm-5");
            elFirst.classList.remove("col-sm-4");

            // other panels
            if(elTwoSt ==0 && elThreeSt==0){
                elTwo.classList.add("col-sm-5");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-5");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");

                dimCam2Width = elTwo.clientWidth;
                dimCam3Width = elThree.clientWidth;
                viewer2.canvas.width= dimCam2Width;
                viewer3.canvas.width= dimCam3Width;
                viewer2.width=dimCam2Width;
                viewer3.width=dimCam3Width;
              //  viewer2.emit('resize');
            }else if(elTwoSt ==1 && elThreeSt==0 ){
                elThree.classList.add("col-sm-10");
                elThree.classList.remove("col-sm-5");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");

                dimCam3Width = elThree.clientWidth;
                viewer3.canvas.width= dimCam3Width;
                viewer3.width=dimCam3Width;
            }else if(elTwoSt ==0 && elThreeSt==1){
                elTwo.classList.add("col-sm-10");
                elTwo.classList.remove("col-sm-5");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");

                dimCam2Width = elTwo.clientWidth;
                viewer2.canvas.width= dimCam2Width;
                viewer2.width=dimCam2Width;
            }else if(elTwoSt ==1 && elThreeSt==1 ){

           //     alert('4');
            }
            break;
        case 'cameraSide':
            elTwoSt = 1;
            elTwo.classList.add("col-sm-1");
            elTwo.classList.remove("col-sm-10");
            elTwo.classList.remove("col-sm-5");
            elTwo.classList.remove("col-sm-4");

            // other panels
            if(elFirstSt ==0 && elThreeSt==0){
                elFirst.classList.add("col-sm-5");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-5");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");

                dimCam1Width = elFirst.clientWidth;
                dimCam3Width = elThree.clientWidth;
                viewer1.canvas.width= dimCam1Width;
                viewer3.canvas.width= dimCam3Width;
                viewer1.width=dimCam2Width;
                viewer3.width=dimCam3Width;
              //  viewer2.emit('resize');
            }else if(elFirstSt ==1 && elThreeSt==0 ){
                elThree.classList.add("col-sm-10");
                elThree.classList.remove("col-sm-5");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");

                dimCam3Width = elThree.clientWidth;
                viewer3.canvas.width= dimCam3Width;
                viewer3.width=dimCam3Width;
            }else if(elFirstSt ==0 && elThreeSt==1){
                elFirst.classList.add("col-sm-10");
                elFirst.classList.remove("col-sm-5");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");

                dimCam1Width =elFirst.clientWidth;
                viewer1.canvas.width= dimCam1Width;
                viewer1.width=dimCam1Width;
            }else if(elFirstSt ==1 && elThreeSt==1 ){

           //     alert('4');
            }
            break;
        case 'cameraHead':
            elThreeSt =1;
            elThree.classList.add("col-sm-1");
            elThree.classList.remove("col-sm-10");
            elThree.classList.remove("col-sm-5");
            elThree.classList.remove("col-sm-4");

            // other panels
            if(elFirstSt ==0 && elTwoSt==0){
                elFirst.classList.add("col-sm-5");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");

                elTwo.classList.add("col-sm-5");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");

                dimCam1Width = elFirst.clientWidth;
                dimCam2Width = elTwo.clientWidth;
                viewer1.canvas.width= dimCam1Width;
                viewer2.canvas.width= dimCam2Width;
                viewer1.width=dimCam1Width;
                viewer2.width=dimCam2Width;
              //  viewer2.emit('resize');
            }else if(elFirstSt ==1 && elTwoSt==0 ){
                elTwo.classList.add("col-sm-10");
                elTwo.classList.remove("col-sm-5");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");

                dimCam2Width = elTwo.clientWidth;
                viewer2.canvas.width= dimCam2Width;
                viewer2.width=dimCam2Width;
            }else if(elFirstSt ==0 && elTwoSt==1){
                elFirst.classList.add("col-sm-10");
                elFirst.classList.remove("col-sm-5");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");

                dimCam1Width =elFirst.clientWidth;
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
                elFirst.classList.add("col-sm-4");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-5");
                elFirst.classList.remove("col-sm-1");

                elTwo.classList.add("col-sm-4");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-5");
                elTwo.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-4");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-5");
                elThree.classList.remove("col-sm-1");

              //  viewer2.emit('resize');
            }else if(elTwoSt ==1 && elThreeSt==0 ){
                elFirst.classList.add("col-sm-5");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-5");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");

            }else if(elTwoSt ==0 && elThreeSt==1){
                elFirst.classList.add("col-sm-5");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");

                elTwo.classList.add("col-sm-5");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");

            }else if(elTwoSt ==1 && elThreeSt==1 ){
                elFirst.classList.add("col-sm-10");
                elFirst.classList.remove("col-sm-5");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");
            }


            break;
        case 'cameraSide':
            elTwoSt = 0;

            // other panels
            if(elFirstSt ==0 && elThreeSt==0){
                elFirst.classList.add("col-sm-4");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-5");
                elFirst.classList.remove("col-sm-1");

                elTwo.classList.add("col-sm-4");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-5");
                elTwo.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-4");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-5");
                elThree.classList.remove("col-sm-1");
            }else if(elFirstSt ==1 && elThreeSt==0 ){
                elTwo.classList.add("col-sm-5");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-5");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");

            }else if(elFirstSt ==0 && elThreeSt==1){
                elFirst.classList.add("col-sm-5");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");

                elTwo.classList.add("col-sm-5");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");

            }else if(elFirstSt ==1 && elThreeSt==1 ){
                elTwo.classList.add("col-sm-10");
                elTwo.classList.remove("col-sm-5");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");
            }
            break;
        case 'cameraHead':;
            elThreeSt =0;

            // other panels
            if(elFirstSt ==0 && elTwoSt==0){
                elFirst.classList.add("col-sm-4");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-5");
                elFirst.classList.remove("col-sm-1");

                elTwo.classList.add("col-sm-4");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-5");
                elTwo.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-4");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-5");
                elThree.classList.remove("col-sm-1");
            }else if(elFirstSt ==1 && elTwoSt==0 ){
                elTwo.classList.add("col-sm-5");
                elTwo.classList.remove("col-sm-10");
                elTwo.classList.remove("col-sm-4");
                elTwo.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-5");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");

            }else if(elFirstSt ==0 && elTwoSt==1){
                elFirst.classList.add("col-sm-5");
                elFirst.classList.remove("col-sm-10");
                elFirst.classList.remove("col-sm-4");
                elFirst.classList.remove("col-sm-1");

                elThree.classList.add("col-sm-5");
                elThree.classList.remove("col-sm-10");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");

            }else if(elFirstSt ==1 && elTwoSt==1 ){
                elThree.classList.add("col-sm-10");
                elThree.classList.remove("col-sm-5");
                elThree.classList.remove("col-sm-4");
                elThree.classList.remove("col-sm-1");
            }
            break;
    }

                dimCam1Width =elFirst.clientWidth;
                dimCam2Width = elTwo.clientWidth;
                dimCam3Width = elThree.clientWidth;
                viewer1.canvas.width= dimCam1Width;
                viewer2.canvas.width= dimCam2Width;
                viewer3.canvas.width= dimCam3Width;
                viewer1.width=dimCam1Width;
                viewer2.width=dimCam2Width;
                viewer3.width=dimCam3Width;
}
    $(window).resize(function(){
        dimCam1Width = elFirst.clientWidth;
        dimCam2Width = elTwo.clientWidth;
        dimCam3Width = elThree.clientWidth;
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