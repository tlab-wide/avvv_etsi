# Configure analyser

After launching the UI, the first stage is running the analyser. To run the analyser on your input data, after [preparing the input data](../../preparing_input/index.md), follow the below steps:

- On the "Analyser" tab, first fill the ROSBAGs directory in the Analyser tab. To do so, click on the browse button in the ROSBAG section and locate the root `rosbag` folder where all RSU and OBU ROSBAG folders reside (you can try with the sample data provided). After selecting the path, a list of all ROSBAG files with their relative paths will appear in the panel.

![UI Analyser tab with the ROSBAG folder selected and ROSBAG files displayed](../../../assets/images/analyser_rosbag.png)

- Then fill in `pcap` directory. Similarly, click on browse in the PCAP section and locate the folder where the PCAP files exist. After choosing the right location, a list of all PCAP files will turn up in the panel.

![UI Visualiser tab with the PCAP folder selected and PCAP files displayed](../../../assets/images/analyser_pcap.png)

- Click on the browse button in the output section. Select a folder where you want your output files generate. The application will create another folder named "output" under the directory you select.

![UI Analyser tab with the output folder selected](../../../assets/images/analyser_output.png)

- Finally, to run the Analyser with your data. Wait till the application finishes. This will generate a single ROSBAG file along with all the graphs and report files.

![UI Analyser tab with the Analyse button clicked](../../../assets/images/analyser_analyse.png)

If anything went wrong, please consult the [troubleshooting page](../../../support/troubleshooting.md).
