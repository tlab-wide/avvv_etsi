from datetime import datetime
from ros2_interface.ros2msg_gen import CPM
from ros2_interface.ros2file_read import tf_type_reader
from general_tools import euclidean_distance
import numpy as np
from plotting import Plotter
from config_avvv import Conf
from rosbags.typesys.types import builtin_interfaces__msg__Time as builtin_time
from rosbags.typesys.types import tf2_msgs__msg__TFMessage as TFMsg
from rosbags.typesys.types import cpm_ros_msgs__msg__NetworkStatus as netstat
from rosbags.typesys.types import cpm_ros_msgs__msg__CPMN as cpmn
from packet_interface import cpm_interface
from packet_interface import pcap_file_interface
from typing import Dict, List


class NetworkStatus:
    """

    """
    sender_station_id: str
    receiver_station_id: str

    sender_packets: list
    receiver_packets: list

    rsu_position_x: float
    rsu_position_y: float

    def __init__(self, sender_station_id: str, receiver_station_id: str, sender_packets: list, receiver_packets: list):
        """

        :param sender_station_id:
        :param receiver_station_id:
        :param sender_packets:
        :param receiver_packets:
        :return:
        """

        self.sender_packets = sender_packets

        self.receiver_packets = receiver_packets

        self.sender_station_id = sender_station_id

        self.receiver_station_id = receiver_station_id

        # converting position of rus in x,y and getting that
        self.rsu_position_x, self.rsu_position_y = cpm_interface.get_rsu_position_in_xy(sender_packets[0])

        self.pair_packet_list_with_delay = self.get_pair_packt_list_with_delay()

        # self.jitter_list = self.measuring_jitter_rfc3550() # todo : this is not completed

        # self.rssi_list = self.measuring_rssi()  # todo : this is not completed

        self.position_netstat_dict = NetworkStatus.creating_position_netstat_dict(self.pair_packet_list_with_delay,
                                                                                  self.receiver_station_id)

        self.delete_far_packets()

        print("size packets after delete : ", len(self.pair_packet_list_with_delay))

        self.plotter = Plotter(self.get_plots_directory())

        self.network_status_list: List[netstat] = self.creating_ros2type_network_status()

        self.cpmn_list = self.creating_ros2type_cpmn()

        if Conf.position_reporter:
            self.plotting_position_graphs()

        if Conf.distance_reporters:
            self.plotting_distance_graphs()  # todo: getting distance list with converting this function to two function

        # if Conf.time_reporter: # todo: this is not completed
        #     self.plotting_time_graphs()

    def get_topic(self, topic_name: str):
        """

        :param topic_name: this is name of topic
        :return:
        """
        topic = "/" + self.receiver_station_id + "/" + self.sender_station_id + "/" + topic_name

        return topic

    def get_plots_directory(self):
        """
        this method return name of plots directory , address is in path_to_output/output/graphs/plots_directory_name
        :return:
        """

        plots_directory_name = self.sender_station_id + "_" + self.receiver_station_id

        return plots_directory_name

    def get_pair_packet_list(self):
        """

        :return: pair packet list withot delay
        """
        sender_pkts_len = len(self.sender_packets)
        receiver_pkts_len = len(self.receiver_packets)

        pair_packet_list = pair_packets(self.sender_packets, self.receiver_packets, sender_pkts_len,
                                        receiver_pkts_len)
        return pair_packet_list

    def get_pair_packt_list_with_delay(self):
        """

        :return: pair packet list with delay
        """

        pair_packet_list = self.get_pair_packet_list()

        pair_packet_list_with_delay = NetworkStatus.measuring_delay_time(pair_packet_list)

        return pair_packet_list_with_delay

    def get_ros2type_network_status_list(self):
        """

        :return:
        """
        return self.network_status_list

    def get_ros2type_cpmn_list(self):
        return self.cpmn_list

    def creating_ros2type_cpmn(self):
        """

        :return:
        """
        ros2type_cpmn_list = []

        for pair in self.pair_packet_list_with_delay:
            send_packet = pair[0]

            rec_packet = pair[1]

            delay = pair[2]

            cpm = CPM(send_packet).get_cpm()

            network_status = NetworkStatus.creating_ros2type_network_status_per_packet(rec_packet, send_packet, delay)

            tf_message = self.position_netstat_dict[
                cpm_interface.get_generationdeltatime(send_packet, False)].tf_message

            cpmn_msg = cpmn(cpm=cpm, network_status=network_status, obu_tf=tf_message)

            msg_info = [cpm_interface.get_epochtime(send_packet), cpmn_msg]

            ros2type_cpmn_list.append(msg_info)

        return ros2type_cpmn_list

    def measuring_jitter_rfc3550(self) -> list:
        """
        If Si is the RTP timestamp from packet i, and Ri is the time of
        arrival in RTP timestamp units for packet i, then for two packets
        i and j, D may be expressed as

            D(i,j) = (Rj - Ri) - (Sj - Si) = (Rj - Sj) - (Ri - Si)

        The interarrival jitter SHOULD be calculated continuously as each
        data packet i is received from source SSRC_n, using this
        difference D for that packet and the previous packet i-1 in order
        of arrival (not necessarily in sequence), according to the formula

            J(i) = J(i-1) + (|D(i-1,i)| - J(i-1))/16

        :return:list of jitter for received packets
        """

        def jitter_formula(previous_jitter: float, d: float) -> float:
            """
            J(i) = J(i-1) + (|D(i-1,i)| - J(i-1))/16
            :param previous_jitter:
            :param d:
            :return:
            """
            return previous_jitter + (abs(d) - previous_jitter) / 16

        jitter_list = [0]  # output list

        delay_list = []
        for i in range(len(self.receiver_packets)):
            #
            # getting packets
            rec_pkt = self.receiver_packets[i]
            sent_pkt = self.get_rec_correspond_pkt(self.sender_packets, rec_pkt)
            #
            # calculating delay
            delay = NetworkStatus.calculate_delay(sent_pkt, rec_pkt)
            #
            delay_list.append(delay)
            #
            #
            if i == 0:
                continue
            #
            # calculating D with above formula
            D = delay_list[i] - delay_list[i - 1]
            #
            # calculating jitter with above formula
            jitter = jitter_formula(jitter_list[i - 1], D)
            #
            jitter_list.append(jitter)

        return jitter_list

    def plotting_time_graphs(self) -> None:
        """
        netstat - time graphs creator
        :return:
        """
        # for netstat_msg in self.network_status_list:
        # netstat(netstat_msg)
        pass

    def plotting_distance_graphs(self):
        """

        :return:
        """
        # todo : adding jitter and rssi to this function

        time_list = []
        distance_list = []
        delay_list = []
        packet_loss_list = []

        # this list is using for distance-delay ,distance-packetLoss and ...
        dicts_list = []

        for pnc in self.position_netstat_dict.values():
            # calculating distance
            distance = euclidean_distance((pnc.x_tf, pnc.y_tf), (self.rsu_position_x, self.rsu_position_y))

            # adding to lists
            time_list.append(cpm_interface.get_epochtime(pnc.rsu_packet))
            distance_list.append(distance)
            delay_list.append(pnc.delay)
            packet_loss_list.append(pnc.packet_loss)

            # adding to dicts_list
            dicts_list.append(dict(distance=distance, delay=pnc.delay, packet_loss=pnc.packet_loss))

        # removing None values
        delay_array = np.array(delay_list)
        delay_array[delay_array == None] = 0
        delay_list = delay_array.tolist()

        # packet loss is boolean change that to (0,1)
        packet_loss_list = np.array(packet_loss_list, dtype=int)

        self.plotter.create_animated_3d_linear_plot_with_matplotlib(np.array(distance_list),
                                                                    np.array(range(len(time_list))),
                                                                    np.array(delay_list),
                                                                    "Delay & Packet loss x Distance x Time (animation)",
                                                                    "distance", "time", "delay", linelabel="delay",
                                                                    pointlabel="packet-loss")

        self.plotter.create_3d_linear_plot_with_matplotlib(np.array(distance_list),
                                                           np.array(range(len(time_list))),
                                                           np.array(delay_list),
                                                           "Delay & Packet loss x Distance x Time ",
                                                           "distance", "time", "delay")

        sorted_list = sorted(dicts_list, key=lambda x: x['distance'])
        sorted_list_distances = []
        sorted_list_packetLoss = []
        sorted_list_delay = []
        for dic in sorted_list:
            sorted_list_distances.append(dic["distance"])
            sorted_list_delay.append(dic["delay"])
            sorted_list_packetLoss.append(dic["packet_loss"])

        # removing None values
        delay_array = np.array(sorted_list_delay)
        delay_array[delay_array == None] = 0
        sorted_list_delay = delay_array.tolist()

        # packet loss is boolean change that to (0,1)
        sorted_list_packetLoss = np.array(sorted_list_packetLoss, dtype=int)

        self.plotter.create_animated_3d_linear_plot_with_matplotlib(np.array(sorted_list_distances),
                                                                    np.zeros_like(sorted_list_distances),
                                                                    np.array(sorted_list_delay),
                                                                    "Delay & Packet loss x Distance ( animation )",
                                                                    "distance", "y", "delay", linelabel="delay",
                                                                    pointlabel="packet-loss")

        self.plotter.create_3d_linear_plot_with_matplotlib(np.array(sorted_list_distances),
                                                           np.zeros_like(sorted_list_distances),
                                                           np.array(sorted_list_delay),
                                                           "Delay & Packet loss x Distance",
                                                           "distance", "y", "delay")

        y_data_list = [np.linspace(0, 0, len(sorted_list_distances)),
                       np.linspace(1, 1, len(sorted_list_distances)),
                       np.linspace(2, 2, len(sorted_list_distances))]

        self.plotter.create_animated_3d_linear_plots_with_matplotlib(sorted_list_distances, y_data_list,
                                                                     [np.linspace(0, 0, len(sorted_list_distances)),
                                                                      sorted_list_delay, sorted_list_packetLoss],
                                                                     "Delay-PacketLoss & Jitter & RSSI x Distance ( animation )",
                                                                     "jitter",
                                                                     "delay", "packet loss",
                                                                     "distance", "y", "value")

        self.plotter.create_3d_linear_plots_with_matplotlib(sorted_list_distances, y_data_list,
                                                            [np.linspace(0, 0, len(sorted_list_distances)),
                                                             sorted_list_delay, sorted_list_packetLoss],
                                                            "Delay-PacketLoss & Jitter & RSSI x Distance",
                                                            "jitter",
                                                            "delay", "packet loss",
                                                            "distance", "y", "value")

    @staticmethod
    def calculate_delay(sent_packet, received_packet) -> float:
        """

        :param sent_packet:
        :param received_packet:
        :return:
        """
        try:
            sent_packet_epoch_time = cpm_interface.get_epochtime(sent_packet)
            received_packet_epoch_time = cpm_interface.get_epochtime(received_packet)

            delay = received_packet_epoch_time - sent_packet_epoch_time

        except:
            delay = 0

        return delay

    def delete_far_packets(self) -> None:
        """
        Deleting packets that obu distance from rsu is more than the maximum distance
        (just for packet loos , maximum distance is in config file)
        :return:
        """
        temp_pair_packet_list_with_delay = []

        for pair in self.pair_packet_list_with_delay:

            # if we don't have packet loss

            # todo:If you want to delete only the packets that are lost and are not in the range, uncomment this section
            # if pair[1] is not None:
            # temp_pair_packet_list_with_delay.append(pair)
            # continue

            # getting position of rsu
            rsu_pkt = pair[0]
            rsu_position = cpm_interface.get_rsu_position_in_xy(rsu_pkt)

            # getting position of obu
            key = cpm_interface.get_generationdeltatime(rsu_pkt, False)
            position_netstat: PositionNetworkStatus = self.position_netstat_dict[key]
            obu_position = (position_netstat.x_tf, position_netstat.y_tf)

            if euclidean_distance(rsu_position, obu_position) > Conf.rsu_effective_distance:
                del self.position_netstat_dict[key]
            else:
                temp_pair_packet_list_with_delay.append(pair)

        self.pair_packet_list_with_delay = temp_pair_packet_list_with_delay

    @staticmethod
    def get_rec_correspond_pkt_with_filename(file_name: str, input_packet):
        """
        this function get input packet and find correspond packet from correspond file
        :param file_name: name of correspond file
        :param input_packet:
        :return: correspond packet of input packet from correspond file
        """
        packet_id = cpm_interface.get_id(input_packet)

        return pcap_file_interface.packet_id_dict[file_name][packet_id]

    @staticmethod
    def measuring_delay_time(pair_pkts_list):
        """

        :param draw:
        :param pair_pkts_list:
        :return:
        """
        for pair_list in pair_pkts_list:
            send_pkt = pair_list[0]
            rec_pkt = pair_list[1]
            if rec_pkt is None:
                pair_list.append(None)
                continue

            rec_time = datetime.fromtimestamp(float(rec_pkt.frame_info.time_epoch))
            send_time = datetime.fromtimestamp(float(send_pkt.frame_info.time_epoch))

            delay = rec_time - send_time

            # print(delay.total_seconds())
            pair_list.append(delay.total_seconds())

        return pair_pkts_list

    @staticmethod
    def measuring_jitter_time(delays_list: list, draw=False) -> float:
        """

        :param delays_list:
        :param draw:
        :return:
        """
        delays = np.array(delays_list)
        jitter = np.std(delays)
        if draw:
            print("Jitter is : " + str(jitter))
            
        return jitter

    @staticmethod
    def creating_ros2type_network_status_per_packet(rec_pkt, send_pkt, delay: float = None):
        """
        this function creating network status ros2 type for just one packet
        :param delay:
        :param rec_pkt:
        :param send_pkt:
        :return:
        """
        if rec_pkt is not None:
            epochtime_rec = cpm_interface.get_epochtime(rec_pkt)

        epochtime_send = cpm_interface.get_epochtime(send_pkt)

        builtin_epochtime = builtin_time(sec=int(str(epochtime_send / 1000000000).split(".")[0]),
                                         nanosec=int(str(epochtime_send / 1000000000).split(".")[1]))

        jitter = 0  # TODO : jitter is not completed
        rssi = 255  # TODO : rssi is not completed

        if rec_pkt is None:
            packet_loss = 1
            delay = -1

        else:
            packet_loss = 0
            if delay is None:
                delay = (epochtime_rec - epochtime_send) / 1000000000

        return netstat(stamp=builtin_epochtime, delay=delay, jitter=jitter, rssi=rssi,
                       packet_loss=packet_loss, packet_count=1)

    def creating_ros2type_network_status(self) -> List[netstat]:
        """
        1.creating some dictionary like { sec value 1 : [ delay0 (in sec value) , ... , delay(n) ] , ... }
        2.crating network status list like : [ [ epochtime average,netstat ] , ... ]
        :return:
        """
        # this is first packet time (send time)
        first_pkt_time = cpm_interface.get_epochtime(self.pair_packet_list_with_delay[0][0])

        #
        #
        #
        # key's are sec times and values are list of delays on that time
        network_status_delay_dict = {}

        # key's are sec times and values are list of epochtimes
        network_status_epochtime_dict = {}

        # key's are sec times and values are packet loss
        network_status_packetloss_dict = {}

        #
        #
        #
        # iteration per pair packets list and adding information to dictionaries
        for pair in self.pair_packet_list_with_delay:

            sender_epochtime = cpm_interface.get_epochtime(pair[0], False)  # epoch time of sender packet

            pkt_time_from_start = ((sender_epochtime - first_pkt_time) / 1000000000)

            key = int(pkt_time_from_start / Conf.network_status_time)

            delay = pair[2]

            try:
                network_status_epochtime_dict[key].append(sender_epochtime)
            except:
                network_status_epochtime_dict[key] = [sender_epochtime]

            if pair[1] is None:  # if we have no delay time . ( we have packet loss )
                try:
                    network_status_packetloss_dict[key].append(1)
                except:
                    network_status_packetloss_dict[key] = [1]

            else:  # if we have delay . ( we don't have packet loss)
                try:
                    network_status_packetloss_dict[key].append(0)
                except:
                    network_status_packetloss_dict[key] = [0]

                try:
                    network_status_delay_dict[key].append(delay)
                except:
                    network_status_delay_dict[key] = [delay]

        #
        #
        #
        # creating network status list ( list : [ [epochtime,netstat()] .... ] )
        network_status_list = []
        #
        #
        # initialization lists for plotting
        delay_list = []
        pkt_loss_list = []
        jitter_list = []
        rssi_list = []
        packet_count_list = []
        epoch_time_avg_list = []

        for key in network_status_epochtime_dict.keys():

            #
            #
            # defining variables
            epochtime_avg: float
            packet_loss_avg: float
            delay_avg: float
            jitter: float
            rssi: int = 255
            packet_count: int

            #
            #
            # initialization of variables
            epochtime_avg = np.average(network_status_epochtime_dict[key])
            packet_loss_avg = np.average(network_status_packetloss_dict[key])
            packet_count = len(network_status_epochtime_dict[key])

            if key in network_status_delay_dict.keys() and len(network_status_delay_dict[key]) != 0:
                delay_avg = np.average(network_status_delay_dict[key])
                jitter = NetworkStatus.measuring_jitter_time(delays_list=network_status_delay_dict[key], draw=False)
            else:
                delay_avg = -1.0
                jitter = -1.0

            builtin_epochtime = builtin_time(sec=int(str(epochtime_avg / 1000000000).split(".")[0]),
                                             nanosec=int(str(epochtime_avg / 1000000000).split(".")[1]))

            #
            #
            # adding to output list
            network_status_list.append([epochtime_avg,
                                        netstat(stamp=builtin_epochtime, delay=delay_avg, jitter=jitter, rssi=rssi,
                                                packet_loss=packet_loss_avg, packet_count=packet_count)])

            #
            #
            # plotting lists
            if Conf.time_reporter:
                delay_list.append(delay_avg)
                jitter_list.append(jitter)
                pkt_loss_list.append(packet_loss_avg)
                rssi_list.append(rssi)
                packet_count_list.append(packet_count)
                epoch_time_avg_list.append(epochtime_avg)

        # plotting
        if Conf.time_reporter:
            self.draw_network_status(Conf.network_status_time, delay_list, jitter_list, pkt_loss_list,
                                     rssi_list,
                                     packet_count_list, epoch_time_avg_list, self.plotter)

        return network_status_list

    @staticmethod
    def draw_network_status(sec_value: int, delay_list: list, jitter_list: list, pkt_loss_list: list, rssi_list: list,
                            packet_count: list, epoch_time_avg_list: list, plotter: Plotter) -> None:
        """

        :param sec_value:
        :param delay_list:
        :param jitter_list:
        :param pkt_loss_list:
        :param rssi_list:
        :param packet_count:
        :param epoch_time_avg_list:
        :param plotter
        :return:
        """

        # creating some lists that needed
        delay_list_normal = np.interp(delay_list, (min(delay_list), max(delay_list)), (0, 1))
        jitter_list_normal = np.interp(jitter_list, (min(jitter_list), max(jitter_list)), (0, 1))
        x = [i for i in range(len(epoch_time_avg_list))]

        y_data_list = [np.linspace(0, 0, len(x)), np.linspace(1, 1, len(x)), np.linspace(2, 2, len(x))]

        # create_animated_3d_linear_plots(x, [delay_list, jitter_list, pkt_loss_list], z_data,
        #                                 "Online delay-jitter-packetLoss 3D graph", "delay", "jitter",
        #                                 "packet-loss", "time", "value", "z")

        plotter.create_animated_3d_linear_plots_with_matplotlib(x, y_data_list,
                                                                [pkt_loss_list, delay_list, jitter_list],
                                                                " Delay & Packet loss & Jitter & RSSI x Time (Real-time per " + str(
                                                                    Conf.network_status_time) + " second)",
                                                                "packet-loss",
                                                                "delay", "jitter",
                                                                "time", "y", "value")

        plotter.create_animated_3d_linear_plots_with_matplotlib(x, y_data_list,
                                                                [pkt_loss_list, delay_list_normal, jitter_list_normal],
                                                                " normal-Delay & Packet loss & normal-Jitter & RSSI x Time (Real-time per " + str(
                                                                    Conf.network_status_time) + " second)",
                                                                "packet-loss",
                                                                "delay", "jitter",
                                                                "time", "y", "value")

        plotter.simple_draw(x, packet_count, "time(sec)", "Packets per " + str(Conf.network_status_time) + " second",
                            "Packet-count X Time ( per " + str(Conf.network_status_time) + " second )")
        plotter.simple_draw(x, delay_list, "time(sec)", "avg. delay (sec)",
                            "Avg. delay X Time ( per " + str(Conf.network_status_time) + " second )")
        plotter.simple_draw(x, jitter_list, "time(sec)", "avg. jitter (sec)",
                            "Avg. jitter X Time ( per " + str(Conf.network_status_time) + " second )")
        plotter.simple_draw(x, pkt_loss_list, "time(sec)", "packet loss ratio (%)",
                            "Packet loss ratio x Time ( per " + str(Conf.network_status_time) + " second )")
        plotter.simple_draw(x, rssi_list, "time(sec)", "RSSI",
                            "RSSI x Time  ( per " + str(Conf.network_status_time) + " second )")
    
    @staticmethod
    def creating_position_netstat_dict(pair_pkts_list, obu_station_id):
        """
        this function creating position reporter
        :param pair_pkts_list:
        :param obu_station_id:
        :return:
        """

        #
        #
        # finding obu ros2 file with obu_station_id
        obu_ros2_file_address = Conf.ros2_files_directory + "/" + obu_station_id

        #
        #
        # dictionary with key = (message stamp time) and value = tf message
        tf_messages = tf_type_reader(rosbag_folder_path=obu_ros2_file_address)

        #
        #
        # this dictionary holding network status of each position with key=time and value PositionNetworkStatus class
        position_netstat_dict = {}

        for pair in pair_pkts_list:

            # getting key of position netstat dictionary( key is generation time of rsu packet)
            key = cpm_interface.get_generationdeltatime(pair[0], False)

            #
            # getting time of packet ( it is time of receiver (obu) except when we have packet loss )
            if pair[1] is None:
                pkt_time = cpm_interface.get_epochtime(pair[0])
            else:
                pkt_time = cpm_interface.get_epochtime(pair[1])

            # pkt_time = cpm_interface.get_epochtime(pair[0]) # todo: another formula

            #
            # finding the closest time in tf messages to time of packet
            temp_min = Conf.max_difference_time_for_equivalent_tf_message

            for tf_message_time in tf_messages:
                if abs(pkt_time - tf_message_time) < temp_min:
                    rsu_packet = pair[0]
                    obu_packet = pair[1]
                    delay = pair[2]
                    tf_message = tf_messages[tf_message_time]
                    temp_min = abs(pkt_time - tf_message_time)

            try:
                position_netstat_dict[key] = PositionNetworkStatus(rsu_packet, obu_packet, tf_message, delay)
            except:
                print("this packet has no message on tf topic")

        return position_netstat_dict


    def plotting_position_graphs(self) -> None:
        """
        this method creates graphs according to each position of obu
        graphs are for packet loss, delay, jitter, and RSSI
        graphs are real-time and static
        :return:
        """
        title_appendix = self.get_plots_directory()

        position_packetLoss_graph(
            self.position_netstat_dict,
            self.plotter,
            self.rsu_position_x,
            self.rsu_position_y,
            title_appendix)

        position_delay_graph(
            self.position_netstat_dict,
            self.plotter,
            self.rsu_position_x,
            self.rsu_position_y,
            title_appendix)

        # position_jitter_graph(self.position_netstat_dict, self.plotter, self.rsu_position_x, self.rsu_position_y, title_appendix) # todo: not completed

        # position_rssi(self.position_netstat_dict, self.plotter, self.rsu_position_x, self.rsu_position_y, title_appendix) # todo: not completed


class PositionNetworkStatus:
    """

    """
    packet = None
    tf_message: TFMsg
    rsu_station_id: str
    obu_station_id: str

    epoch_time: float  # it is obu epoch time except when we have packet loss
    tf_msg_time: float

    rsu_station_position = None

    x_tf: float
    y_tf: float
    z_tf: float

    delay: float
    packet_loss: bool
    jitter: float
    rssi: float

    def __init__(self, rsu_packet, obu_packet, tf_message: TFMsg, delay: float):
        """

        :param rsu_packet:
        :param obu_packet:
        :param tf_message:
        :return:
        """
        self.tf_message = tf_message
        self.rsu_packet = rsu_packet
        self.obu_packet = obu_packet
        self.delay = delay
        self.x_tf, self.y_tf, self.z_tf = self.get_tf_translation()

        if obu_packet is None:
            self.packet_loss = True
        else:
            self.packet_loss = False

    def get_tf_translation(self):
        """

        :param msg:
        :return: (x,y,z)
        """
        x = self.tf_message.transforms[0].transform.translation.x
        y = self.tf_message.transforms[0].transform.translation.y
        z = self.tf_message.transforms[0].transform.translation.z

        return x, y, z


def pair_packets(send_cap, rec_cap, send_cap_len, rec_cap_len):
    """
    this function pair corresponding packets on the sender and receiver sides
    :return: 2d list . example : [ [sender_pkt1,receiver_pkt1],...,[sender_pkt(n),receiver_pkt(n)] ]
    """

    print("packet loss : " + str(send_cap_len - rec_cap_len))

    pair_corresponding_pkts = []

    # temporary dictionary for holding received packets
    rec_pkt_dict: dict = {}
    for index in range(rec_cap_len):
        key = cpm_interface.get_id(rec_cap[index])
        if key not in rec_pkt_dict.keys():
            rec_pkt_dict[key] = rec_cap[index]
        else:
            print("There are some packets with the same Rsu StationID and packet ID in the packets")
            raise ValueError("There are some packets with the same Rsu StationID and packet ID in the packets")

    # iteration on sender packet for finding corresponding packet in the receiver packets
    for index in range(send_cap_len):
        send_pkt = send_cap[index]
        send_pkt_id = cpm_interface.get_id(send_pkt)

        if send_pkt_id in rec_pkt_dict.keys():
            rec_pkt = rec_pkt_dict[send_pkt_id]
        else:
            rec_pkt = None

        pair_corresponding_pkts.append([send_pkt, rec_pkt])

        if rec_pkt is not None:
            del rec_pkt_dict[cpm_interface.get_id(send_pkt)]

    return pair_corresponding_pkts


def position_packetLoss_graph(
        position_netstat_dict: Dict[float, PositionNetworkStatus],
        plotter: Plotter,
        rsu_x_position: float,
        rsu_y_position: float,
        title_appendix: str) -> None:
    """
    this method creating position X packet-loss graphs
    :return:
    """

    x = []
    x_loss = []
    y = []
    y_loss = []
    z = []
    z_loss = []

    x_grid = []
    y_grid = []
    z_grid = []
    loss_grid = []

    for position_time in position_netstat_dict:
        position_netstat_class: PositionNetworkStatus = position_netstat_dict[position_time]

        x_grid.append(position_netstat_class.x_tf)
        y_grid.append(position_netstat_class.y_tf)
        z_grid.append(position_netstat_class.z_tf)

        if position_netstat_class.packet_loss:
            loss_grid.append(1)
            x_loss.append(position_netstat_class.x_tf)
            y_loss.append(position_netstat_class.y_tf)
            z_loss.append(position_netstat_class.z_tf)

        else:
            loss_grid.append(0)
            x.append(position_netstat_class.x_tf)
            y.append(position_netstat_class.y_tf)
            z.append(position_netstat_class.z_tf)

    z_loss = [0] * len(z_loss)  # this line removing z values from packets

    plotter.creating_3d_scatter(x_loss, y_loss, z_loss, [1] * len(z_loss), rsu_x_position=rsu_x_position,
                                rsu_y_position=rsu_y_position,
                                boolean_point=True,
                                title="Packet-loss X Position(" + title_appendix + ")",
                                size_of_points=Conf.packet_loss_point_size,
                                color_of_points=Conf.packet_loss_color)

    plotter.griding_points(x_grid, y_grid, z_grid, loss_grid, rsu_x_position, rsu_y_position,
                           rgba_number=1,
                           title="Packet-loss rate x Position (" + str(
                               Conf.position_reporter_grid_x_size) + "m grid)(" + title_appendix + ")",
                           x_label="x-axis"
                           , y_label="y-axis", z_label="Packet Loss")


def position_delay_graph(position_netstat_dict: Dict[float, PositionNetworkStatus], plotter: Plotter,
                         rsu_x_position: float,
                         rsu_y_position: float,
                         title_appendix: str) -> None:
    """
    this method creating delay X position graphs
    :return:
    """

    delay_list = [position_netstat_dict[position_time].delay for position_time in position_netstat_dict]
    x = [position_netstat_dict[position_time].x_tf for position_time in position_netstat_dict]
    y = [position_netstat_dict[position_time].y_tf for position_time in position_netstat_dict]
    z = [position_netstat_dict[position_time].z_tf for position_time in position_netstat_dict]

    z = [0] * len(z)  # removing z

    # packet to packet delay graph
    plotter.creating_3d_scatter(x, y, z, delay_list, rsu_x_position, rsu_y_position,
                                size_of_points=Conf.position_delay_packet_point_size,
                                title="Delay x Position(" + title_appendix + ")")

    # grid graphs ( histogram )
    plotter.griding_points(x, y, z, delay_list, rsu_x_position, rsu_y_position, rgba_number=100,
                           title="Delay x Position (" + str(
                               Conf.position_reporter_grid_x_size) + "m grid)(" + title_appendix + ")",
                           x_label="x-axis", y_label="Y-axis", z_label="delay value")

    # replacing None values with 0 in delay list
    tmp_delay_list = [0 if item is None else item for item in delay_list]

    # position-delay real-time graph
    plotter.create_animated_3d_linear_plot_with_matplotlib(np.array(x), np.array(y), np.array(tmp_delay_list),
                                                           " Delay & Packet loss x Position (Real-time)(" + title_appendix + ")",
                                                           "x",
                                                           "y",
                                                           "delay", linelabel="delay",
                                                           pointlabel="packet loss")
