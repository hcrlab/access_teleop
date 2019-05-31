/**
 * Created by timadamson on 8/22/17.
 */
// Edited by Xinyi

Head = function (ros) {

    // Set up the freeze publishers
    var head = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/head_z',
        messageType: 'access_teleop_msgs/HeadZ'
    });

    // A function for publishing 'z' to access_teleop_msgs/HeadZ
    // negative z: look down
    // positive z: look up
    this.tilt = function (z){
        var tiltZ = new ROSLIB.Message({
            tilt: z
        });
        head.publish(tiltZ);
    };

    this.adv = function() {
        head.advertise();
    };

    this.unadv = function() {
        head.unadvertise();
    };

};