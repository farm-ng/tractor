# rtkbase application

## Introduction

This is the application for running a simple ublox M8T receiver + raspberry pi (or other embedded computer).
Its meant to be matched with the farm-ng/tractor so the configuration is minimal and meant to be plug and play.


### balena Setup & Deployment

To get this project up and running, you will need to signup for a balena account [here][signup-page] and set up an application and device.

Create a balina application called "rtkbasestation", and add a device to the application. You'll find full details in Balena's [Getting Started tutorial][gettingStarted-link].


Then you can `balena push rtkbasestation` from this directory to push the app to the device, see balena's documentation here [balenaCLI][balena-cli]. This command will package up and push the code to the balena builders, where it will be compiled and built and deployed to the device you added in balina. When it completes, you can check the logs on your [balenaCloud dashboard][balena-dashboard].

[balena-link]:https://balena.io/
[signup-page]:https://dashboard.balena-cloud.com/signup
[gettingStarted-link]:http://balena.io/docs/learn/getting-started/
[balena-cli]:https://www.balena.io/docs/reference/cli/
[balena-dashboard]:https://dashboard.balena-cloud.com/
