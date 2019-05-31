/**
 * Created by Xinyi on 05/09/19.
 */

Base = function (ros) {

    var baseTopic = new ROSLIB.Topic({
        ros: ros,
        name: 'cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    // Set up the base publishers
    var task = new ROSLIB.Topic({
        ros: ros,
        name: '/access_teleop/task_type',
        messageType: 'access_teleop_msgs/TaskType'
    });


    this.goToPrev = function(taskNum) {
        var prevTask = new ROSLIB.Message({
            task_type: 0
        });
        task.publish(prevTask);
    };

    this.goToNext = function(taskNum) {
        var nextTask = new ROSLIB.Message({
            task_type: 1
        });
        task.publish(nextTask);
    };

    this.adv = function() {
        baseTopic.advertise();
        task.advertise();
    };

    this.unadv = function() {
        baseTopic.unadvertise();
        task.unadvertise();
    };
};

function move(direction, topic) {
    var lv = 0;
    var rv = 0;
    if (direction == "forward") {
        lv = 0.5;
    } else if (direction == "back") {
        lv = -0.5;
    } else if (direction == "left") {
        rv = 1;
    } else {
        rv = -1;
    }
    topic.publish({
        linear : {
            x: lv, // Set positive or negative meters/s to drive
            y: 0,
            z: 0
        },
        angular : {
            x: 0,
            y: 0,
            z: rv // Set rads/s to turn
        }
    });
}