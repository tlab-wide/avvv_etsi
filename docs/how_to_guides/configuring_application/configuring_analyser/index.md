# Configure analyser

After launching the UI, the first stage is running the analyser. To run the analyser on your input data, after [preparing the input data](../../preparing_input/index.md), follow the below steps:

## Input ROSBAG

On the "Analyser" tab, first fill the ROSBAGs directory in the Analyser tab. To do so, click on the browse button in the ROSBAG section and locate the root `rosbag` folder where all RSU and OBU ROSBAG folders reside (you can try with the sample data provided). After selecting the path, a list of all ROSBAG files with their relative paths will appear in the panel.

<div>
    <img src="../../../assets/images/analyser_rosbag.png" alt="UI Analyser tab with the ROSBAG folder selected and ROSBAG files displayed" class="configure_img"/>
</div>

## Input PCAP

Then fill in `pcap` directory. Similarly, click on browse in the PCAP section and locate the folder where the PCAP files exist. After choosing the right location, a list of all PCAP files will turn up in the panel.

<div>
    <img src="../../../assets/images/analyser_pcap.png" alt="UI Visualiser tab with the PCAP folder selected and PCAP files displayed" class="configure_img"/>
</div>

## Output Directory

Click on the browse button in the output section. Select a folder where you want your output files generate. The application will create another folder named "output" under the directory you select.

<div>
    <img src="../../../assets/images/analyser_output.png" alt="UI Analyser tab with the output folder selected" class="configure_img"/>
</div>

## Run the Analyser

Finally, to run the Analyser with your data. Wait till the application finishes. This will generate a single ROSBAG file along with all the graphs and report files.

<div>
    <img src="../../../assets/images/analyser_analyse.png" alt="UI Analyser tab with the Analyse button clicked" class="configure_img"/>
</div>

If anything went wrong, please consult the [troubleshooting page](../../../support/troubleshooting.md).
