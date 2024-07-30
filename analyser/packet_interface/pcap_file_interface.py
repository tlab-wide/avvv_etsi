from pyshark import FileCapture

# this dictionary save captured packets
pcap_to_cap_dict = {}  # dict with key= pcap file name , and value= captured of pcap file

# this dictionary save packets with packet id
packet_id_dict = {}  # dict with key = pcap file name , and value = dict with key = packet_id, value = packet


def reading_file(file_name: str, file_directory_address: str):
    """

    :param file_directory_address:
    :param file_name:
    :return:
    """
    # print("reading file : ", pcap_to_cap_dict.keys())
    if file_name in pcap_to_cap_dict.keys():
        cap, cap_length = pcap_to_cap_dict.get(file_name)
        return cap, cap_length

    # Name of tcpdump file (.pcap file)
    pcap_file = file_directory_address + "/" + file_name
    print("reading from ", pcap_file)

    # Import using pyshark (Python wrapper of tshark)
    cap = FileCapture(pcap_file, display_filter='its', use_json=True, include_raw=False)

    # getting number of packets
    cap_length = len([p for p in cap])

    cap.close()

    pcap_to_cap_dict[file_name] = (cap, cap_length)

    return cap, cap_length


def find_rsu_file_from_station_id(station_id: str) -> str:
    """
    this function find rsu pcap file with station id
    :param station_id:
    :return:
    """
    # TODO: this function is not completed yet because we have just one rsu file and that name is not standard
    return "20221214_1645_wlo1_rsu.pcap"


def pcap_files_reporter(obu_file_names: list, rsu_file_names: list) -> None:
    """
    this function gets a list of pcap files and creates a report file for them
    :param obu_file_names:
    :param rsu_file_names:
    :return:
    """
    for obu_file in obu_file_names:
        for rsu_file in rsu_file_names:

            file_name = "./output_files/report_" + str(obu_file).split(".")[0] + "_" + str(rsu_file).split(".")[
                0] + "_files.txt"

            with open(file_name, 'w') as f:

                obu_cap, obu_cap_len = reading_file(obu_file)
                rsu_cap, rsu_cap_len = reading_file(rsu_file)

                f.write("number of ITS packets in " + str(obu_file) + "(obu) is: " + str(obu_cap_len) + "\n")
                f.write("number of ITS packets in " + str(rsu_file) + "(rsu) is: " + str(rsu_cap_len) + "\n\n")

                f.write("----------\n")
                f.write("ID = generationTime_timestamp(gnw)_rsuStationID\n\n")

                f.write("number of different packet id in " + str(obu_file) + "(obu) is: " + str(
                    len(packet_id_dict[obu_file])) + "\n")
                f.write("number of different packet id in " + str(rsu_file) + "(obu) is: " + str(
                    len(packet_id_dict[rsu_file])) + "\n\n")

                f.write("----------\n")

                packets_in_obu_not_in_rsu = 0
                for id in packet_id_dict[obu_file].keys():
                    if id not in packet_id_dict[rsu_file].keys():
                        packets_in_obu_not_in_rsu += 1
                f.write("number of packets that are in obu but not in rsu is: " + str(packets_in_obu_not_in_rsu) + "\n")
