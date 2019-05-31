/**
 * Created by timadamson on 9/5/17.
 */

CloudFreezer = function (ros) {

    // Set up the freeze publishers
    var freezer = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/freeze_cloud',
        messageType: 'std_msgs/Bool'
    });

    this.adv = function() {
        freezer.advertise();
    };

    this.unadv = function() {
        freezer.unadvertise();
    };

    // A function for publishing 'True' to /access_teleop/freeze_cloud
    this.freezeCloud = function () {
        var freeze = new ROSLIB.Message({
            data : true
        });
        freezer.publish(freeze);
    };

    this.unfreezeCloud = function (){
        var unfreeze = new ROSLIB.Message({
            data: false
        });
        freezer.publish(unfreeze);
    };

};