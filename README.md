# Blinky
A web-based touchscreen face for HRI research.
It has a ROS interface for changing the face and calling behaviors.
Blinky is primarily designed and tested for Chrome on a Nexus 7 (2013) tablet.

![blinky](https://cloud.githubusercontent.com/assets/1175286/12600875/baf9204c-c451-11e5-98f5-7fbaa8b57a9e.png)

## Getting started
- Install the latest version of node:

  ```
  curl -sL https://deb.nodesource.com/setup_5.x | sudo -E bash -
  sudo apt-get install -y nodejs
  ```
- Clone this repository to your catkin workspace and build it.
- From the `frontend` folder, run `npm install -g gulp bower && npm install && bower install`
- Run `roslaunch rosbridge_server rosbridge_websocket.launch`
- Run `gulp serve` and go to `localhost:5001` to see the face.
- It's recommended that you add the app to the device's homescreen, so that the app can be run full-screen.
  In Chrome, tap the menu button in the top right (the "three dots") and tap "Add to Home screen."
  On Android 5.0 (Lollipop) and above, you can additionally pin the app using Screen Pinning to hide the notification bar.

You should see a blank face.
To send a message to it, send a goal to its actionlib server, `/blinky`, of type `blinky/FaceAction`:
```
rostopic pub -1 blinky/goal blinky/FaceActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  display_type: 'displayMessage'
  h1_text: 'Hello world!'
  h2_text: 'How can I help you?'"
```

## Deploying
- Install Apache: `sudo apt-get install apache2`

Note this section is still in development.

## Behaviors
### displayMessage(h1_text, h2_text)
This will make the robot display a message to communicate to users.
The robot can display big text (for legibility at a distance), or just regular text (to fit more content on screen), or a combination of both.

More behaviors coming soon!
