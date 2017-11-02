/**
 * Created by timadamson on 8/31/17.
 * This file creates the gripper's ui. It uses a double slider to make both fingers, and has then both move when one is moved
 */

var $slider;


GripperGUI = function (gripper_controler) {
    this.$slider = $slider;
    var self = this;

    var max = 0.1;
    var min = 0.0;

    this.$slider.slider({
        range: true,
        min: min,
        max: max,
        step: 0.001,
        values:[min, max],
        slide: function( event, ui ) {
            var otherFinger = ui.handleIndex === 0 ? 1 : 0;
            self.$slider.slider("values", otherFinger, max - ui.values[ui.handleIndex]);
        },
        stop: function (event, ui) {
            var newPos = ui.values[1] - ui.values[0];
            gripper_controler.moveGripper(newPos, gripper_controler.MAX_EFFORT);
        }
    });

    //This function will adjust the gripper slider on the page if they exist
    this.adjustGUI = function(position){
        position = position.toFixed(3);
        if(document.getElementById("slider-range")){
            var leftFinger = (max - min - position) / 2;
            var rightFinger = max - leftFinger;
            //console.log("The left Finger position is " + leftFinger);
            self.$slider.slider("values", 0, leftFinger ); // Second value is 0 because we are moving the left gripper
            self.$slider.slider("values", 1, rightFinger ); // Second value is 0 because we are moving the left gripper
        }
        else{
            console.error("The slider does not exist")
        }
    };

};

