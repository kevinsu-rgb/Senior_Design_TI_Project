# log
### 1/12->1/19: 9 hours.
- Run Pose and Fall Detection Example + Motion and Presence Detection + Understand Radar Setup Code  (5 hours.)
- Troubleshoot mmWave Data Recorder to save data in csv (4 hours.)
### 1/19->1/26: 11 hours.
- Research Data Visualization for heatmap data using matplotlib (2 hours.)
- Research existing TI ML model to understand the dependencies they use and
  also take note of how to write efficient, legible, code in Jupyter Notebook.
  (3 hours.)
- Setup Jupyter Notebook with PyTorch and dataset imports (4 hours.)
- Continue working on Radar configuration to begin dataset collection (2 hours.)
### 1/26->2/02: 16 hours.
- Work on creating synthetic range azimuth heatmap data via Python script (3 hours.)
- Work on heatmap config, debug config, attempt to collect heatmap data, notice bug with TI data recorder (3 hours.)
- Met up with team at CpE lab to debug radar heatmap data collection. Try out different configs, edit data recorder source code, schedule meeting with Pedrhom (3 hours.)
- Discuss cfg file with Pedrhom on 1/30 in meeting from 12-1 PM. (1 hour.)
- Got sick with food poisoning on 1/30 from Panda Express... rested 1/30 and 1/31
- Worked on collecting heatmap data and visualizing this data on 2/01. Created json to csv converter that normalized data and collected sitting and standing range azimuth data (3 hours.)
- Continued testing model on range azimuth data on 2/02. (3 hours.)
### 2/02->2/09: 11 hours.
- Met up with Team after the meeting at CpE lab. Worked on getting range doppler working on diff demo, 2/04. (3 hours.)
- Worked on collecting range doppler data. Default config wasn't working. Messaged Pedrhom for help. Was able to log range doppler data, 2/05. (3 hours.)
- Worked on processing the range doppler data into 2D array, and visualizing it. Troubleshooted # of range bins and # of doppler bins. Worked on UART reader with Jaiden to switch from azimuth to doppler, 2/07. (3 hours.)
- Worked with Pedrhom to optimize cfg settings, because doppler visualization wasn't very clear. (2 hours.)
### 2/09->2/16: 11 hours.
- Met up with TI to discuss project status. Determined SPI would be a good option. (1 hour.)
- Worked on getting SPI data capture to work. (1 hour.)
- Continued working on SPI data capture, was unsucessful and decided to move back to UART data collection. (3 hours.)
- Pivoted to collecting point cloud + heatmap data. Added point cloud functionality to our UART reader, read.py. (4 hours.)
- Worked on adding tracker information to read.py, which contains accel, vel, and pos. (2 hours.)
### 2/16->2/20: 7 hours.
- Met up with team at Marston. Discussed beta and worked on formatting the csv for optimal tensor input into model. (2 hours.)
- Worked on changing the way we format csv. (2 hours.)
- Worked on updating config for better range in data collection. (1 hour.)
- Worked on demoing the radar in CpE lab, still having issues with range. (2 hours.)
### Total hours: 65 hours.
