Ryan Sauer, 12:53 PM
Alright, once your ready here's the task:
1. Get the SkySocket folder from the github I just uploaded. It contains the files for the GUI
2. Download this zip, it contains an embedded python with all of the dependencies you'll need
3. Unzip the embeded python into the skysocket folder
4. Run a command from the skysocket folder: <embeded-python>/python.exe __main__.py and you should see the gui appear

Ryan Sauer, 12:56 PM
Task number 2 is to test the video with the drones video:
1. Open skysocket.kv
2. Edit line 7 so VIDEO_IP's string is the address for the video stream
3. Edit line 95 to have state: 'play'
4. Start the video stream on the drone via SSH
5. Run the GUI
If the stream shows up, that's perfect, if not, copy the terminal output. there should be a whole bunch of debug logging. take a screenshot for me and tell me whats up

Ryan Sauer, 1:02 PM
Task number 3 is to test the ssh stuff:
1. open __init__.py in the ssh folder
2. populate all of the globals. They should be named aptly so you understand what they're for. Make sure the directory and file names are correct
3. call start_experiment() under everything (line 29 or 30)
4. Make sure the drone and car are on
5. open a cmd and run <embeded-python>/python.exe ssh/__init__.py
Let me know if it doesnt work or what output is given. It's pretty readable code, Im jsut making a connection, moving to a directory, then calling a command. There is a chance I should wait to do close(), so if it doesnt work and you're feeling like debugging, try deleting car_con.close() and drone_con.close()
