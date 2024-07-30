from general_tools import create_directory, merge_dicts
from network_status import *
from abc import ABC
from plotting import Plotter


class Node(ABC):
    """
    this class is parent class for the rsu and abu class
    obu and rsu classes inherit from this class
    """

    def __init__(self, pcap_file_name: str):
        self._pcap_file_name: str = pcap_file_name
        self._packets: List = self.__reading_pcap_file()
        self._packets_id_dict: Dict = self.__creating_packet_id_dict()

    def __reading_pcap_file(self) -> List:
        cap, _ = pcap_file_interface.reading_file(file_name=self._pcap_file_name, file_directory_address=Conf.pcap_files_directory)
        return cap

    def __creating_packet_id_dict(self) -> Dict:
        """
        this method creating a dictionary with key = id of packet and value = packet
        :return: packet_id_dict
        """
        packets_id_dict: dict = {}
        for pkt in self._packets:
            packets_id_dict[cpm_interface.get_id(pkt)] = pkt

        return packets_id_dict

    def get_packets(self) -> List:
        return self._packets

    def get_pcap_file_name(self) -> str:
        return self._pcap_file_name

    def get_packet_id_dict(self) -> Dict:
        return self._packets_id_dict


class Rsu(Node):
    def __init__(self, pcap_file_address: str):
        super().__init__(pcap_file_address)
        self.__rsu_position_x, self.__rsu_position_y = cpm_interface.get_rsu_position_in_xy(self.get_packets()[0])
        self.__rsu_station_id = cpm_interface.get_station_id(self.get_packets()[0])
        self.__network_statuses: List[NetworkStatus] = []
        self.__plotter = Plotter(self.get_plots_directory_name())

    def __netstat_position_graphs(self) -> None:
        """
        this method creates network status position graphs for this Rsu
        :return:
        """
        # getting all NetworkStatusPosition's dictionaries of this Rus
        netstat_position_dicts: List[Dict[float, PositionNetworkStatus]] = self.get_netstat_position_dicts()

        # merging all NetworkStatusPosition's dictionaries to one NetworkStatusPosition dictionary
        netstat_position_dicts_merged = merge_dicts(netstat_position_dicts, overwrite=False, important_keys=False)

        #
        # creating graphs

        position_packetLoss_graph(netstat_position_dicts_merged, self.__plotter, self.__rsu_position_x,
                                  self.__rsu_position_y, self.get_plots_directory_name())

        position_delay_graph(netstat_position_dicts_merged, self.__plotter, self.__rsu_position_x,
                             self.__rsu_position_y, self.get_plots_directory_name())

    def graphs(self) -> None:
        """
        this method creates all graphs of this Rsu
        :return:
        """
        self.__netstat_position_graphs()

    def append_network_status(self, netstat_obj: NetworkStatus) -> None:
        """
        this method for adding new network status object to the list of network statuses
        :param netstat_obj: NetworkStatus object
        :return:
        """
        self.__network_statuses.append(netstat_obj)

    def get_plots_directory_name(self) -> str:
        """
        this method creates and return the name of the plots directory of this Rsu in the graphs directory
        :return:
        """
        return "RSU_" + str(self.__rsu_station_id)

    def get_netstat_position_dicts(self) -> List[Dict[float, PositionNetworkStatus]]:
        """
        this method return list of position network status dictionaries
        :return:
        """
        return [netstat_obj.position_netstat_dict for netstat_obj in self.__network_statuses]

    def get_network_status_objects(self) -> List[NetworkStatus]:
        """
        this method return list of NetworkStatus objects
        :return: NetworkStatus objects list
        """
        return self.__network_statuses

    def get_position(self):
        """
        this method return position of rsu station in x and y
        :return:
        """
        return self.__rsu_position_x, self.__rsu_position_y

    def get_station_id(self):
        """
        this method return station id of Rsu
        :return:
        """
        return self.__rsu_station_id


class Obu(Node):
    def __init__(self, pcap_file_address: str):
        super().__init__(pcap_file_address)


class NodesManager:
    """
    this class creates and manages all the Rsu and Obu classes
    """

    def __init__(self, obu_file_names: List[str], rsu_file_names: List[str]):
        self.__obu_file_names: List[str] = obu_file_names
        self.__rsu_file_names: List[str] = rsu_file_names
        self.__rsu_nodes: List[Rsu] = self.__create_rsu_nodes()
        self.__obu_nodes: List[Obu] = self.__create_obu_nodes()
        self.__nodes_reporter()

    def __create_rsu_nodes(self) -> List[Rsu]:
        """
        this method creates rsu objects from rsu file names
        :return:
        """
        rsu_nodes_list: List[Rsu] = []

        for rsu_pcap_file_name in self.__rsu_file_names:
            rsu_nodes_list.append(Rsu(rsu_pcap_file_name))

        return rsu_nodes_list

    def __create_obu_nodes(self) -> List[Obu]:
        """
        this method creates obu objects from obu file names
        :return:
        """
        obu_nodes_list: List[Obu] = []

        for obu_pcap_file_name in self.__obu_file_names:
            obu_nodes_list.append(Obu(obu_pcap_file_name))

        return obu_nodes_list

    def __nodes_reporter(self) -> None:
        """
        This method creates a file report of nodes and saves it in the output directory
        :return:
        """
        # creating directory of Nodes_Reporters
        directory_address = Conf.report_output_directory_address + "/Nodes_Reports"
        create_directory(directory_address)

        for rsu in self.__rsu_nodes:
            for obu in self.__obu_nodes:
                full_file_address = directory_address + "/" + rsu.get_pcap_file_name().split(".")[0] + "-" + \
                                    obu.get_pcap_file_name().split(".")[0] + ".txt"

                with open(full_file_address, 'w') as f:
                    obu_cap = obu.get_packets()
                    rsu_cap = rsu.get_packets()

                    f.write("Packets Report\n---------\n\n")
                    f.write("number of ITS packets in " + obu.get_pcap_file_name() + " (obu) is: " + str(
                        len(list(obu_cap))) + "\n")
                    f.write("number of ITS packets in " + rsu.get_pcap_file_name() + " (rsu) is: " + str(
                        len(list(rsu_cap))) + "\n\n")

                    f.write("---------\n")
                    f.write("ID = generationTime_timestamp(gnw)_rsuStationID\n\n")

                    f.write("number of different packet id in " + obu.get_pcap_file_name() + " (obu) is: " + str(
                        len(obu.get_packet_id_dict())) + "\n")
                    f.write("number of different packet id in " + rsu.get_pcap_file_name() + " (rsu) is: " + str(
                        len(rsu.get_packet_id_dict())) + "\n\n")

                    f.write("---------\n")

    def get_obu_nodes(self) -> List[Obu]:
        return self.__obu_nodes

    def get_rsu_nodes(self) -> List[Rsu]:
        return self.__rsu_nodes

    def get_obu_file_names(self) -> List[str]:
        return self.__obu_file_names

    def get_rsu_file_names(self) -> List[str]:
        return self.__rsu_file_names
