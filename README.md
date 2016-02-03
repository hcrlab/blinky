# Blinky
A web-based touchscreen face for HRI research.
It has a ROS interface for changing the face and calling behaviors.
Blinky is primarily designed and tested for Chrome on a Nexus 7 (2013) tablet.

![blinky](https://cloud.githubusercontent.com/assets/1175286/12600875/baf9204c-c451-11e5-98f5-7fbaa8b57a9e.png)

## Getting started
- Install the latest version of node:

  ```bash
  curl -sL https://deb.nodesource.com/setup_5.x | sudo -E bash -
  sudo apt-get install -y nodejs
  ```
- Clone this repository to your catkin workspace and build it.
- From the `frontend` folder, run `sudo npm install -g gulp bower` and then `npm install && bower install`
- Run `roslaunch rosbridge_server rosbridge_websocket.launch`
- Run `gulp serve` and go to `localhost:5001` to see the face.
- It's recommended that you add the app to the device's homescreen, so that the app can be run full-screen.
  In Chrome, tap the menu button in the top right (the "three dots") and tap "Add to Home screen."
  On Android 5.0 (Lollipop) and above, you can additionally pin the app using Screen Pinning to hide the notification bar.

You should see a blank face.
To send a message to it, send a goal to its actionlib server, `/blinky`, of type `blinky/FaceAction`:
```bash
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

These instructions describe how to deploy Blinky to an Apache server running on the robot computer.
We assume that a Nexus 7 tablet is connected to the laptop via USB.

### Build the website
```bash
cd frontend
gulp
sudo cp -r dist /var/www/blinky
```

### Set up Apache
Install Apache if you don't already have it: `sudo apt-get install apache2`

In our setup, we are going to serve the website from port 8080.
The URL will be something like `robot.university.edu:8080/blinky`
This allows us to serve the website from the robot's computer via USB, thanks to Chrome's port forwarding / mobile device debugging feature.

See DigitalOcean's [guide to configuring Apache](https://www.digitalocean.com/community/tutorials/how-to-configure-the-apache-web-server-on-an-ubuntu-or-debian-vps) for a great primer on configuring Apache.

Create the file /etc/apache2/blinky:
```ApacheConf
Listen 8080
<VirtualHost *:8080>
  ServerAdmin webmaster@localhost
  ServerName robot.university.edu
  ServerAlias robot
  ServerAlias localhost

  DocumentRoot /var/www
  <Directory />
    Options FollowSymLinks
    AllowOverride None
    Order Deny,Allow
    Deny from All
  </Directory>

  <Directory /var/www/blinky>
    Options Indexes FollowSymLinks MultiViews
    AllowOverride None
    Order allow,deny
    allow from all
  </Directory>

  ErrorLog ${APACHE_LOG_DIR}/error.log

  # Possible values include: debug, info, notice, warn, error, crit,
  # alert, emerg.
  LogLevel warn

  CustomLog ${APACHE_LOG_DIR}/access.log combined
</VirtualHost>
```

Enable the blinky site using `sudo a2ensite blinky`.
Then, restart the Apache service with `sudo service apache2 reload`.

### Load the page on the tablet
You should be able to access the webpage from the regular internet by going to `robot.university.edu:8080/blinky`.
However, we want the face to be robust to loss of wifi.
Instead, we load the page directly from the robot's computer over USB.

To do this, open an SSH connection to the robot's computer using the `-X` option and run `google-chrome`
```bash
ssh -X robot
google-chrome
```

On the tablet, enable USB debugging (see [Remote Debugging Devices](https://developers.google.com/web/tools/chrome-devtools/debug/remote-debugging/remote-debugging?hl=en)).
From Chrome, go to `chrome://inspect`.
You should see the Nexus 7 in the device list.
Go to port forwarding and forward port 8080 to `localhost:8080` and port 9090 to `localhost:9090`, which is necessary for the rosbridge websocket connection (see [Remote Access to Your Local Site](https://developers.google.com/web/tools/chrome-devtools/debug/remote-debugging/local-server?hl=en)).

## Behaviors
### displayMessage(h1_text, h2_text)
This will make the robot display a message to communicate to users.
The robot can display big text (for legibility at a distance), or just regular text (to fit more content on screen), or a combination of both.

### askMultipleChoice(question, choices)
This will make the robot ask a question on screen.
`choices` is a list of strings, each of which will be displayed as a button.
The choice the user selects will be returned as the actionlib result.
If there is only one choice, the button will be colored, otherwise they are grey.

More behaviors coming soon!
