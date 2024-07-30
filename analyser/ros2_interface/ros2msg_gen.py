"""
This module creates ros2 messages
"""
from packet_interface import cpm_interface
from rosbags.typesys.types import builtin_interfaces__msg__Time as builtin_time
from rosbags.typesys.types import cpm_ros_msgs__msg__PerceivedObject as PerceivedObject
from rosbags.typesys.types import cpm_ros_msgs__msg__ReferencePosition as ReferencePosition
from rosbags.typesys.types import cpm_ros_msgs__msg__CPMMessage as CpmMessage
from rosbags.typesys.types import cpm_ros_msgs__msg__PerceivedObjectContainer as PerceivedObjectContainer
from rosbags.typesys.types import cpm_ros_msgs__msg__CollectivePerceptionMessage as CollectivePerceptionMessage
from rosbags.typesys.types import cpm_ros_msgs__msg__CPMManagementContainer as CpmManagementContainer
from rosbags.typesys.types import cpm_ros_msgs__msg__CPMParameters as CPMParameters
from rosbags.typesys.types import cpm_ros_msgs__msg__ItsPduHeader as ItsPduHeader
from rosbags.typesys.types import cpm_ros_msgs__msg__StationDataContainer as StationDataContainer
from rosbags.typesys.types import cpm_ros_msgs__msg__OriginatingRSUContainer as OriginatingRSUContainer
from rosbags.typesys.types import cpm_ros_msgs__msg__OriginatingVehicleContainer as OriginatingVehicleContainer


def creating_ros2type_builtin_interfaces_Time(pkt):
    """

    :param pkt:
    :return:
    """
    time_epoch = pkt.frame_info.time_epoch
    sec = int(str(time_epoch).split(".")[0])
    nanosec = int(str(time_epoch).split(".")[1])

    return builtin_time(sec=sec, nanosec=nanosec)


class CPM:

    def __init__(self, packet):
        self.packet = packet
        self.__cpm = self.__cpm_creator()

    def get_cpm(self) -> CpmMessage:
        """
        :return:
        """
        return self.__cpm

    def __cpm_creator(self) -> CpmMessage:
        """
        this method creating cpm message in ros2
        :return:
        """
        stamp = creating_ros2type_builtin_interfaces_Time(self.packet)
        header = self.__its_pdu_header()
        cpm = self.__collective_perception_message()
        return CpmMessage(stamp=stamp, header=header, cpm=cpm)

    def __its_pdu_header(self) -> ItsPduHeader:
        """

        :return:
        """
        station_id: int = int(cpm_interface.get_station_id(self.packet))
        protocol_version = 1
        message_id = 14
        return ItsPduHeader(protocol_version=protocol_version, message_id=message_id, station_id=station_id)

    def __collective_perception_message(self) -> CollectivePerceptionMessage:
        """

        :return:
        """
        generationDeltaTime: int = int(cpm_interface.get_generationdeltatime(self.packet, print_bool=False))
        cpmParameters = self.__cpm_parameters()

        return CollectivePerceptionMessage(
            generation_delta_time=generationDeltaTime,
            cpm_parameters = cpmParameters)

    def __cpm_parameters(self) -> CPMParameters:
        """

        :return:
        """
        management_container = self.__cpm_management_container()
        station_data_container = self.__station_data_container()
        perceived_object_container = self.__perceived_object_container()
        sensor_information_container = self.__sensor_information_container()
        number_of_PerceivedObjects = self.__number_of_perceived_objects()

        return CPMParameters(
            management_container=management_container,
            station_data_container=station_data_container,
            sensor_information_container=sensor_information_container,
            perceived_object_container=perceived_object_container,
            number_of_perceived_objects=number_of_PerceivedObjects)

    def __cpm_management_container(self) -> CpmManagementContainer:
        """

        :return:
        """
        station_type = int(cpm_interface.get_station_type(self.packet))
        reference_position = self.__reference_position()

        return CpmManagementContainer(
            station_type=station_type,
            reference_position=reference_position)

    def __reference_position(self) -> ReferencePosition:
        """

        :return:
        """
        longitude = float(cpm_interface.get_longitude(self.packet))
        latitude = float(cpm_interface.get_latitude(self.packet))

        return ReferencePosition(longitude=longitude, latitude=latitude)

    def __station_data_container(self) -> StationDataContainer:
        """

        :return:
        """
        originating_vehicle_container = self.__originating_vehicle_container()
        originating_rsu_container = self.__originating_rsu_container()

        return StationDataContainer(
            originating_vehicle_container=originating_vehicle_container,
            originating_rsu_container=originating_rsu_container)

    def __sensor_information_container(self) -> StationDataContainer:
        """
        # TODO: Complete this method
        :return:
        """

        return StationDataContainer(
            originating_vehicle_container=OriginatingVehicleContainer(
                heading=0,
                speed=0),
            originating_rsu_container=OriginatingRSUContainer(
                intersection_reference_id=0,
                road_segment_reference_id=0))

    def __originating_vehicle_container(self):
        """

        :return:
        """
        if cpm_interface.get_originating_vehicle_container(self.packet) is None:
            return OriginatingVehicleContainer(heading=0, speed=0)
        speed = int(cpm_interface.get_origin_vehicle_speed(self.packet))
        heading = int(cpm_interface.get_origin_vehicle_header(self.packet))

        return OriginatingVehicleContainer(heading=heading, speed=speed)

    def __originating_rsu_container(self):
        """

        :return:
        """
        try:
            intersectionReferenceId = int(cpm_interface.get_intersectionReferenceId(self.packet))
            roadSegmentReferenceId = int(cpm_interface.get_roadSegmentReferenceId(self.packet))
        except TypeError:
            intersectionReferenceId = 0
            roadSegmentReferenceId = 0

        return OriginatingRSUContainer(intersection_reference_id=intersectionReferenceId,
                                       road_segment_reference_id=roadSegmentReferenceId)

    @staticmethod
    def __perceived_object(perceivedobject_element):
        """

        :param perceivedobject_element:
        :return:
        """
        try:
            # get values
            objectID = cpm_interface.get_objectID(perceivedobject_element)
            # time_of_measurement = cpm_interface.get_timeOfMeasurement(perceivedobject_element)
            xDistance_value = cpm_interface.get_xDistance_value(perceivedobject_element)
            yDistance_value = cpm_interface.get_yDistance_value(perceivedobject_element)
            xSpeed_value = cpm_interface.get_xSpeed_value(perceivedobject_element)
            ySpeed_value = cpm_interface.get_ySpeed_value(perceivedobject_element)
            yawAngle_value = cpm_interface.get_yawAngle_value(perceivedobject_element)
            planarObjectDimension1_value = cpm_interface.get_planarObjectDimension1_value(perceivedobject_element)
            planarObjectDimension2_value = cpm_interface.get_planarObjectDimension2_value(perceivedobject_element)
            verticalObjectDimension_value = cpm_interface.get_verticalObjectDimension_value(perceivedobject_element)

            # creating perceivedObject ros2type
            return PerceivedObject(
                object_id=objectID,
                x_distance=xDistance_value,
                y_distance=yDistance_value,
                z_distance=0.0,
                x_speed=xSpeed_value,
                y_speed=ySpeed_value,
                z_speed=0.0,
                roll_angle=0.0,
                pitch_angle=0.0,
                yaw_angle=yawAngle_value,
                planar_object_dimension1=planarObjectDimension1_value,
                planar_object_dimension2=planarObjectDimension2_value,
                vertical_object_dimension=verticalObjectDimension_value)
        
        except Exception as e:
            print("can't read the perceivedObject element for this packet . Error Message: ", e)
            return None

    def __perceived_object_container(self):
        """

        :param pkt:
        :return:
        """
        perceivedObjectContainer_list = []

        try:
            poc_tree = cpm_interface.get_perceivedObjectContainer_tree(self.packet)

            for pobj in list(poc_tree._all_fields.keys()):
                perceivedobject_element = poc_tree.get(pobj).PerceivedObject_element
                perceived_object_ros2type = self.__perceived_object(perceivedobject_element)
                perceivedObjectContainer_list.append(perceived_object_ros2type)
        except AttributeError:
            pass
        except Exception as e:
            print("can't read the perceivedObjectContainer_tree for this packet(",
                cpm_interface.get_generationdeltatime(self.packet, print_bool=False), "). Error Message : ", e)

        return PerceivedObjectContainer(perceived_objects=perceivedObjectContainer_list)

    def __number_of_perceived_objects(self):
        """

        :return:
        """
        return int(cpm_interface.get_number_of_perceived_object(self.packet))
