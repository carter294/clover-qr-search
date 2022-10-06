# clover-qr-search
The script forces clover-copter to search for a specific qr-code with a data typed in console and land on it.

First of all you need to type in qr size in qr_size variable.

Also it's desirable to create a new image topic for throttling frames of a main camera, otherwise the script will take up to 100% CPU capacity. Then subscribe to this topic in image_sub variable. (guide is here https://clover.coex.tech/en/camera.html)

In the bottom of the script you can change the path you drone follows searching qr codes.

After launching the script you'll have to input data, which is encrypted in qr code you want to find.
