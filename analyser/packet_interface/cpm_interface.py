from pyshark.packet import packet
from general_tools import lat_long_to_mgrs


def get_originating_rsu_container(pkt: packet.Packet):
    try:
        return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.stationDataContainer_tree.originatingRSUContainer_element
    except Exception as e:
        print("can't read the originating RSU container : \n", e)
        return None


def get_originating_vehicle_container(pkt: packet.Packet):
    try:
        return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.stationDataContainer_tree.originatingVehicleContainer_element
    except Exception as e:
        print("can't read the originating vehicle container : \n", e)
        return None


def get_station_data_container(pkt: packet.Packet) -> int | None:
    try:
        return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.stationDataContainer_tree
    except Exception as e:
        print("can't read the station data container : \n", e)
        return None


def get_number_of_perceived_object(pkt: packet.Packet) -> int | None:
    try:
        return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.numberOfPerceivedObjects

    except Exception as e:
        print("Can't reading the number of perceived objects. ERROR MESSAGE : \n", e)
        return None


def get_origin_vehicle_header(pkt: packet.Packet) -> float | None:
    try:
        return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.stationDataContainer_tree.originatingVehicleContainer_element.heading_element.headingValue
    except Exception as e:
        print("Can't reading the heading of originating vehicle container : \n", e)
        return None


def get_origin_vehicle_speed(pkt: packet.Packet) -> float | None:
    try:
        return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.stationDataContainer_tree.originatingVehicleContainer_element.speed_element.speedValue
    except Exception as e:
        print("Can't reading the speed of originating vehicle container : \n", e)
        return None


def get_intersectionReferenceId(pkt: packet.Packet) -> int | None:
    """

    :param pkt:
    :return:
    """
    try:
        return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.stationDataContainer_tree.originatingRSUContainer_element.intersectionReferenceId_element.intersectionReferenceId
    except AttributeError:
        return None
    except Exception as e:
        print("Can't read intersectionReferenceId of originating RSU container:", e)
        return None


def get_roadSegmentReferenceId(pkt: packet.Packet) -> int | None:
    """

    :param pkt:
    :return:
    """
    try:
        return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.stationDataContainer_tree.originatingRSUContainer_element.roadSegmentReferenceId_element.roadSegmentReferenceId
    except Exception as e:
        print("Can't reading the roadSegmentReferenceId of originating RSU container : \n", e)
        return None


def get_station_type(pkt: packet.Packet) -> int:
    """

    :param pkt:
    :return:
    """
    return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.managementContainer_element.stationType


def get_station_id(pkt: packet.Packet):
    """

    :param pkt:
    :return:
    """
    # todo: this function changed for test ( because station id is not unique )
    # todo: for simulation we using station type instead of station id ( for now )
    # if Conf.simulation:
    #     return get_station_type(pkt)

    # return pkt.its.ItsPduHeader_element.stationID
    return 0


def get_epochtime(pkt, print_bool=False):
    """

    :param pkt:
    :param print_bool:
    :return:
    """
    if print_bool:
        print("Epoch time:", float(pkt.frame_info.time_epoch) * 1000000000)

    return float(pkt.frame_info.time_epoch) * 1000000000


def get_longitude(pkt):
    """

    :param pkt:
    :return:
    """
    return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.managementContainer_element.referencePosition_element.longitude


def get_latitude(pkt):
    """

    :param pkt:
    :return:
    """
    return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.managementContainer_element.referencePosition_element.latitude


def get_yaw(pkt):
    """

    :param pkt:
    :return:
    """
    # todo: this function is not completed
    # return pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.managementContainer_element.referencePosition_element.yaw
    return 0


def get_perceivedObjectContainer_tree(pkt):
    """

    :return:
    """
    try:
        poc_tree = pkt.its.CollectivePerceptionMessage_element.cpmParameters_element.perceivedObjectContainer_tree
    except AttributeError:
        return None
    except Exception as e:
        print("get_perceivedObjectContainer_tree Exception:", e)
        poc_tree = None

    return poc_tree


def get_xDistance_value(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return float(perceivedobject_element.xDistance_element.value)
    except Exception as e:
        print("can't read the xDistance of this perceivedobject_element. Error Message : ", e)
        return None


def get_yDistance_value(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return float(perceivedobject_element.yDistance_element.value)
    except Exception as e:
        print("can't read the yDistance of this perceivedobject_element. Error Message : ", e)
        return None


def get_xSpeed_value(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return float(perceivedobject_element.xSpeed_element.value)
    except Exception as e:
        print("can't read the xSpeed of this perceivedobject_element. Error Message : ", e)
        return None


def get_ySpeed_value(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return float(perceivedobject_element.ySpeed_element.value)
    except Exception as e:
        print("can't read the ySpeed of this perceivedobject_element. Error Message : ", e)
        return None


def get_objectID(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return perceivedobject_element.objectID
    except Exception as e:
        print("can't read the objectID for this perceived object . Error Message: ", e)
        return None


def get_yawAngle_value(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return float(perceivedobject_element.yawAngle_element.value)
    except Exception as e:
        print("can't read the yawAngle for this perceivedobject . Error Message: ", e)
        return None


def get_planarObjectDimension1_value(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return float(perceivedobject_element.planarObjectDimension1_element.value)
    except Exception as e:
        print("can't read the plannerObjectDimension1 for this perceived object . Error Message: ", e)
        return None


def get_planarObjectDimension2_value(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return float(perceivedobject_element.planarObjectDimension2_element.value)
    except Exception as e:
        print("can't read the plannerObjectDimension2 for this perceived object . Error Message: ", e)
        return None


def get_verticalObjectDimension_value(perceivedobject_element):
    """

    :param perceivedobject_element:
    :return:
    """
    try:
        return float(perceivedobject_element.verticalObjectDimension_element.value)
    except Exception as e:
        print("can't read the verticalObjectDimension for this perceived object . Error Message: ", e)
        return None


def get_generationdeltatime(pkt, print_bool=True):
    if print_bool:
        print(pkt.its.CollectivePerceptionMessage_element.generationTime)

    return pkt.its.CollectivePerceptionMessage_element.generationTime


def get_timestamp(pkt, print_bool=True):
    if print_bool:
        print(pkt.gnw.tsb.src_pos_tree.tst)

    return pkt.gnw.tsb.src_pos_tree.tst


def get_id(pkt):
    timestamp = get_timestamp(pkt, False)
    station_id = get_station_id(pkt)
    generation_time = get_generationdeltatime(pkt, False)
    lat = get_latitude(pkt)
    pkt_id = str(generation_time) + "_" + str(timestamp) + "_" + str(station_id) + "_" + str(lat)

    return pkt_id


def get_rsu_position_in_xy(pkt) -> tuple:
    latitude = float(get_latitude(pkt)) / 10 ** 7
    longitude = float(get_longitude(pkt)) / 10 ** 7

    mgrs = lat_long_to_mgrs(latitude=latitude, longitude=longitude)

    y = float(mgrs[-5:])
    x = float(mgrs[-10:-5])
    # x = 3838.893951286046
    # y = 73743.06043979211
    return x, y


def get_cpm(cap, index=0):
    # getting cpm message of packet
    cpm = cap[index].its.CollectivePerceptionMessage_element

    # it is all of cpm parameters and perceived object containers
    cpm_str = str(cpm)

    # getting cpm parameters
    cpm_parameters = cpm.cpmParameters_element

    try:
        pOC = cpm_parameters.perceivedObjectContainer_tree
        # Get perceivedObjects and add to cpm_str

        # perceivedObjects = []
        for pObj in list(pOC._all_fields.keys()):
            # perceivedObjects.append(pOC.get(pObj).PerceivedObject_element)
            cpm_str += str(pOC.get(pObj).PerceivedObject_element)

    except:
        print("this packet has not perceivedObjectContainer_tree")

    return cpm_str
