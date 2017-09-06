/**
 * Created by timadamson on 9/5/17.
 */

$(document).ready(function(){

    var btn1St=0;
    var btn2St=0;
    var btn3St=0;

    $("#top").click(function(){
        $(".collapse1").collapse('toggle');
    });

    $("#side").click(function(){
        $(".collapse2").collapse('toggle');
    });

    $("#head").click(function(){
        $(".collapse3").collapse('toggle');
    });


    function clearClasses() {

        $("#first").removeClass();
        $("#second").removeClass();
        $("#third").removeClass();
    }

    $(".collapse1").on('show.bs.collapse', function(){
        //  alert('The collapsible content is about to be shown.');
        btn1St=0;

        clearClasses();
    });

    $(".collapse2").on('show.bs.collapse', function(){
        //  alert('The collapsible content is about to be shown.');
        btn2St=0;
        clearClasses();
    });

    $(".collapse3").on('show.bs.collapse', function(){
        //  alert('The collapsible content is about to be shown.');
        btn3St=0;
        clearClasses();

    });

    $(".collapse1").on('shown.bs.collapse', function(){
        //alert('The collapsible content is now fully shown.');
        if (btn2St==0  &&  btn3St==0) {
            $("#first").addClass("col-sm-4");
            $("#second").addClass("col-sm-4");
            $("#third").addClass("col-sm-4");
        } else if (btn2St==0  &&  btn3St==1) {
            $("#first").addClass("col-sm-5");
            $("#second").addClass("col-sm-5");
            $("#third").addClass("col-sm-1");
        }else if (btn2St==1  &&  btn3St==0) {
            $("#first").addClass("col-sm-5");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-5");
        } else if (btn2St==1  &&  btn3St==1) {
            $("#first").addClass("col-sm-10");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-1");
        }
    });


    $(".collapse2").on('shown.bs.collapse', function(){
        //alert('The collapsible content is now fully shown.');
        if (btn1St==0  &&  btn3St==0) {
            $("#first").addClass("col-sm-4");
            $("#second").addClass("col-sm-4");
            $("#third").addClass("col-sm-4");
        } else if (btn1St==0  &&  btn3St==1) {
            $("#first").addClass("col-sm-5");
            $("#second").addClass("col-sm-5");
            $("#third").addClass("col-sm-1");
        }else if (btn1St==1  &&  btn3St==0) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-5");
            $("#third").addClass("col-sm-5");
        } else if (btn1St==1  &&  btn3St==1) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-10");
            $("#third").addClass("col-sm-1");
        }
    });


    $(".collapse3").on('shown.bs.collapse', function(){
        //alert('The collapsible content is now fully shown.');
        if (btn1St==0  &&  btn2St==0) {
            $("#first").addClass("col-sm-4");
            $("#second").addClass("col-sm-4");
            $("#third").addClass("col-sm-4");
        } else if (btn1St==0  &&  btn2St==1) {
            $("#first").addClass("col-sm-5");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-5");
        }else if (btn1St==1  &&  btn2St==0) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-5");
            $("#third").addClass("col-sm-5");
        } else if (btn1St==1  &&  btn2St==1) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-10");
        }
    });

    $(".collapse1").on('hide.bs.collapse', function(){
        //alert('The collapsible content is about to be hidden.');
        btn1St=1;
        clearClasses();

    });

    $(".collapse2").on('hide.bs.collapse', function(){
        //alert('The collapsible content is about to be hidden.');
        btn2St=1;
        clearClasses();
    });

    $(".collapse3").on('hide.bs.collapse', function(){
        //alert('The collapsible content is about to be hidden.');
        btn3St=1;
        clearClasses();
    });

    $(".collapse1").on('hidden.bs.collapse', function(){
        //alert('The collapsible content is now hidden.');
        if (btn2St==0  &&  btn3St==0) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-5");
            $("#third").addClass("col-sm-5");
        } else if (btn2St==0  &&  btn3St==1) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-10");
            $("#third").addClass("col-sm-1");
        }else if (btn2St==1  &&  btn3St==0) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-10");
        } else if (btn2St==1  &&  btn3St==1) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-1");
        }
    });


    $(".collapse2").on('hidden.bs.collapse', function(){
        //alert('The collapsible content is now hidden.');
        if (btn1St==0  &&  btn3St==0) {
            $("#first").addClass("col-sm-5");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-5");
        } else if (btn1St==0  &&  btn3St==1) {
            $("#first").addClass("col-sm-10");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-1");
        }else if (btn1St==1  &&  btn3St==0) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-10");
        } else if (btn1St==1  &&  btn3St==1) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-1");
        }
    });

    $(".collapse3").on('hidden.bs.collapse', function(){
        //alert('The collapsible content is now hidden.');
        if (btn1St==0  &&  btn2St==0) {
            $("#first").addClass("col-sm-5");
            $("#second").addClass("col-sm-5");
            $("#third").addClass("col-sm-1");
        } else if (btn1St==0  &&  btn2St==1) {
            $("#first").addClass("col-sm-10");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-1");
        }else if (btn1St==1  &&  btn2St==0) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-10");
            $("#third").addClass("col-sm-1");
        } else if (btn1St==1  &&  btn2St==1) {
            $("#first").addClass("col-sm-1");
            $("#second").addClass("col-sm-1");
            $("#third").addClass("col-sm-1");
        }
    });
});