# Configure Vagrant to use a joystick

By default, vagrant does not enable USB controllers. In fact, by default, there is no way to ask vagrant to enable USB controllers. By enabling USB controllers and installing a few drivers, we can read the output of a joystick, which is connected to the host computer, from inside the virtual machine.

NOTA BENE: These instructions assume you’re using OS X, VirtualBox, and an Xbox360 USB controller.

## 1. Install VirtualBox and VirtualBox Extension Pack
You may already have a VirtualBox version, but install one that lines up with the Extension Pack version from here: https://www.virtualbox.org/wiki/Downloads.

## 2. Install 360Controller
This maps your Xbox360 controller to standard Mac joystick drivers. Get version >=0.15 here: https://github.com/360Controller/360Controller.

Optionally, after the install, go to the app store and install “Controllers Lite”. Plug in your controller and open the app. It should reflect your button presses and joystick motions. Then, unplug the controller.

## 3. Find your controller’s info
Assuming the previous steps were successful, your mac recognizes your Xbox360 controller. Go to Option+Apple > System Information > USB > Controller. Copy the Vendor ID and Product ID attributes.

## 4. Add the following lines to the appropriate location in your Vagrantfile:
```
vb.customize ["modifyvm", :id, "--usb", "on"]
vb.customize ["modifyvm", :id, "--usbehci", "on"]
vb.customize ["usbfilter", "add", "0",
  "--target", :id,
  "--name", "xbox360",
  "--vendorid", “<your vendor id>“,
  "--productid", “<your product id>“]
```

Save the file, then `vagrant reload`. When it’s finished, `vagrant ssh` into the VM.

## 5. Install drivers
* `modprobe joydev`
* `sudo apt-get install joystick xboxdrv`
* Plug in your controller
* At this point, running `lsusb` should display your controller.
* `xboxdrv`
* After some positive stdout about finding the controller, any manual input to the controller will produce representative data in the terminal.

## 6. Configure xboxdrv to start at boot
* `sudoedit /etc/init/xboxdrv.conf`
* Add contents:
```
start on filesystem
exec xboxdrv -D
expect fork
```
* `echo “joydev” | sudo tee —-append /etc/modules’