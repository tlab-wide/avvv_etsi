import time
import threading
import numpy as np
import concurrent.futures
import matplotlib.pyplot as plt
from typing import List


class PointTimeValue:
    __time: float
    __value: float

    def __init__(self, time: float, value: float):
        if time is None or value is None:
            raise ValueError("time or value is not set")
        self.__time = time
        self.__value = value

    def get_time(self) -> float:
        """
        this method returning time of data point
        :return: time of data point
        """
        return self.__time

    def get_value(self) -> float:
        """
        this method returning value of data point
        :return: value of data point
        """
        return self.__value


class Point3D:
    __x: float
    __y: float
    __z: float
    __value: float

    def __init__(self, x: float = None, y: float = None, z: float = None, value: float = None):
        self.__x = x
        self.__y = y
        self.__z = z
        self.__value = value

    def get_x(self) -> float:
        """
        this method return x of point
        :return: x of point
        """
        if self.__x is None:
            raise ValueError("x is not set")
        return self.__x

    def get_y(self) -> float:
        """
        this method return y of point
        :return: x of point
        """
        if self.__y is None:
            raise ValueError("y is not set")
        return self.__y

    def get_z(self) -> float:
        """
        this method return z of point
        :return: x of point
        """
        if self.__z is None:
            raise ValueError("z is not set")
        return self.__z

    def get_value(self) -> float:
        """
        this method return value of point
        :return: x of point
        """
        if self.__value is None:
            raise ValueError("value is not set")
        return self.__value


class PlotDataStore:
    __plot_data_points_3d: List[Point3D]
    __plot_data_points_time_value: List[PointTimeValue]
    __title: str
    __x_name: str
    __y_name: str
    __z_name: str

    def __init__(self, title: str, x_name: str, y_name: str, z_name: str = None, color: str = "blue",
                 directly_appending_data: bool = True,
                 data_queue_address: str = None):
        self.__title = title
        self.__x_name = x_name
        self.__y_name = y_name
        self.__z_name = z_name
        self.__color = color

        self.__plot_data_points_3d: List[Point3D] = []
        self.__plot_data_points_time_value: List[PointTimeValue] = []

        # if data appending indirectly (from outside database)
        if not directly_appending_data:
            self.__add_new_point_indirectly(data_queue_address)

    def get_plot_data_points_3d(self) -> List[Point3D]:
        """
        this method return list of all points for plot
        :return:
        """
        return self.__plot_data_points_3d

    def get_plot_data_points_time_value(self) -> List[PointTimeValue]:
        """
        this method return list of all points for plot
        :return:
        """
        return self.__plot_data_points_time_value

    def get_title(self) -> str:
        """
        this method return title of plot
        :return:
        """
        return self.__title

    def get_x_name(self) -> str:
        """
        this method return x label of plot
        :return:
        """
        return self.__x_name

    def get_y_name(self) -> str:
        """
        this method return x label of plot
        :return:
        """
        return self.__y_name

    def get_z_name(self) -> str:
        """
        this method return z label of plot
        :return:
        """
        return self.__z_name

    def get_color(self) -> str:
        """
        this method return color of plot
        :return:
        """
        return self.__color

    def add_new_3d_point_directly(self, new_point: Point3D) -> None:
        """
        This method is to directly add a 3D new point to the points
        :param new_point:
        :return: None
        """
        self.__plot_data_points_3d.append(new_point)

    def add_new_timevalue_point_directly(self, new_point: PointTimeValue) -> None:
        """
        This method is to directly add a 3D new point to the points
        :param new_point:
        :return: None
        """
        self.__plot_data_points_time_value.append(new_point)

    def __add_new_point_indirectly(self, data_base_address: str) -> None:
        pass  # todo: this method is not completed


class RealTimePlotter:
    __plot_data_stores: List[PlotDataStore]

    def __init__(self, *plot_data_stores: PlotDataStore, figure_name="NetworkStatus - Time"):
        self.__plot_data_stores = []
        for plot_data_stores in plot_data_stores:
            self.__plot_data_stores.append(plot_data_stores)

        self.fig = plt.figure(num=figure_name)
        self.ax = self.fig.subplots(len(self.__plot_data_stores))

    def __simple_time_value_plotter(self) -> None:
        """
        this method drawing simple 'value X time' plot
        :return:
        """
        for index in range(len(self.__plot_data_stores)):
            plot_data_store: PlotDataStore = self.__plot_data_stores[index]
            ax = self.ax[index]
            data_list = plot_data_store.get_plot_data_points_time_value()
            data_value_list = [data.get_value() for data in data_list]
            data_time_list = [data.get_time() for data in data_list]
            ax.plot(data_time_list, data_value_list, marker='.', color=plot_data_store.get_color())
            ax.set_title(plot_data_store.get_title())
            ax.set_xlabel(plot_data_store.get_x_name())
            ax.set_ylabel(plot_data_store.get_y_name())
        plt.pause(0.01)

    def update(self) -> None:
        self.__simple_time_value_plotter()


# pds_delay = PlotDataStore("delay X time", "time", "delay", color="red")
# pds_loss = PlotDataStore("packet loss X time", "time", "packet loss")
# pds_jitter = PlotDataStore("jitter X time","time","Jitter", color="yellow")
# pds_rssi = PlotDataStore("rssi X time", "time", "rssi", color="green")
#
# if __name__ == "__main__":
#     rtp = RealTimePlotter(pds_loss, pds_delay,pds_rssi,pds_jitter)
#     for i in range(100):
#         time.sleep(1)
#         pds_delay.add_new_timevalue_point_directly(PointTimeValue(i, i ** 2))
#         pds_loss.add_new_timevalue_point_directly(PointTimeValue(i, i * 2))
#         pds_rssi.add_new_timevalue_point_directly(PointTimeValue(i,10))
#         pds_jitter.add_new_timevalue_point_directly(PointTimeValue(i,2 ** i))
#         rtp.update()
3