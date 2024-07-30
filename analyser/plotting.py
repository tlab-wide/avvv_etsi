import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
import csv_tools
import seaborn as sns
import tkinter as tk
import matplotlib.colors as mcolors
from config_avvv import Conf
from matplotlib.animation import FuncAnimation
from pathlib import Path


class Plotter:
    plots_directory_name: str  # This is the name of the folder where the charts are stored
    plt_style: str
    showing_graphs: bool
    save_graphs_image: bool
    save_graphs_pdf: bool
    save_graphs_csv: bool
    save_graphs_pickle: bool

    def __init__(self, plots_directory_name: str):
        self.plots_directory_name = plots_directory_name
        self.plt_style = Conf.graphs_style
        self.showing_graphs = Conf.showing_graphs
        self.save_graphs_image = Conf.save_graphs_image
        self.save_graphs_pdf = Conf.save_graphs_pdf
        self.save_graphs_csv = Conf.save_graphs_csv
        self.save_graphs_pickle = Conf.save_graphs_pickle

    def save_plot(self, figure, plot_name: str, animation=None, csv_dict=None) -> None:
        """
        this method saves plots to the plots_directory in the path_to_output/output_files/graphs/plots_directory_name
        :return:
        """

        def create_directory(directory_path: Path) -> None:
            """
            this function creating directory in the directory_path

            :param directory_path: path to directory
            :return:
            """
            if not os.path.exists(directory_path):
                os.makedirs(directory_path)

        def save_plot_in_image(images_directory_path: Path) -> None:
            """
            this function saving plot in image ( .png format )
            :param images_directory_path: path of images directory
            :return:
            """
            figure.savefig(str(images_directory_path) + "/" + plot_name + ".png")

        def save_plot_in_pdf(pdfs_directory_path: Path) -> None:
            """
            this function saving plot in pdf ( .pdf format )
            :param pdfs_directory_path: path of pdfs directory
            :return:
            """
            figure.savefig(str(pdfs_directory_path) + "/" + plot_name + ".pdf", dpi=300, format='pdf')

        def save_plot_in_gif(gifs_directory_path: Path) -> None:
            """
            this function saving plot in gif ( .gif format )
            :param gifs_directory_path: path of gifs directory
            :return:
            """
            animation.save(str(gifs_directory_path) + "/" + plot_name + ".gif", writer='pillow')

        def save_plot_in_pickle(pickles_directory_path: Path) -> None:
            """
            this function saving plot in pickle ( .pickle format )
            :param pickles_directory_path: path of pickles directory
            :return:
            """
            with open(str(pickles_directory_path) + "/" + plot_name + '.pickle', 'wb') as f:
                pickle.dump(figure, f)
                # todo : adding animation saver to this function

        def save_plot_in_csv(csvs_directory_path: Path) -> None:
            """
            this function saving plot in csv file( .csv format )
            :param csvs_directory_path: path of csvs directory
            :return:
            """

            file_absolute_address = str(csvs_directory_path) + "/" + plot_name + ".csv"

            csv_tools.csv_from_dict(csv_dict, file_absolute_address)

        #
        # creating graphs directory in output_files directory
        plots_address = self.get_plots_directory_address()

        create_directory(Path(Conf.report_output_directory_address + "/graphs"))

        create_directory(Path(plots_address))

        #
        # saving in .png format
        if animation is None and self.save_graphs_image:
            images_directory = Path(plots_address + "/images")

            create_directory(Path(images_directory))

            save_plot_in_image(images_directory)

        #
        # saving in .pdf format
        if animation is None and self.save_graphs_pdf:
            pdfs_directory = Path(plots_address + "/pdfs")

            create_directory(pdfs_directory)

            save_plot_in_pdf(pdfs_directory)

        #
        # saving in .pickle format
        if self.save_graphs_pickle:
            pickles_directory = Path(plots_address + "/pickles")

            create_directory(pickles_directory)

            save_plot_in_pickle(pickles_directory)

        #
        # saving in .csv format
        if csv_dict is not None and self.save_graphs_csv:
            csvs_directory = Path(plots_address + "/csvs")

            create_directory(csvs_directory)

            save_plot_in_csv(csvs_directory)

    def get_plots_directory_address(self):
        """
        this method returning the full address of plots directory
        :return:
        """

        plots_directory_address = Conf.report_output_directory_address + "/graphs/" + self.plots_directory_name

        return plots_directory_address

    def griding_points(self, x: list, y: list, z: list, values: list, rsu_x_position: float, rsu_y_position: float,
                       rgba_number=10,
                       histogram: bool = True,
                       title: str = "position reporter",
                       x_label: str = "X-axis",
                       y_label: str = "Y-axis",
                       z_label: str = "Z-axis"):
        """
        :param rsu_x_position
        :param rsu_y_position
        :param rgba_number:
        :param histogram:
        :param x:
        :param y:
        :param z:
        :param values:
        :param title:
        :param x_label
        :param y_label
        :param z_label
        :return:
        """

        def indexing_value(value: float, value_min: float, value_dis: float) -> int:
            return int((value - value_min) / value_dis)

        def get_average_values(points_list: list) -> tuple:
            x_avg = np.average(np.array([point[0] for point in points_list]))
            y_avg = np.average(np.array([point[1] for point in points_list]))
            z_avg = np.average(np.array([point[2] for point in points_list]))
            val_arr = np.array([point[3] for point in points_list if point[3] is not None])
            if val_arr.size > 0:
                value_avg = np.average(val_arr)
            else:
                value_avg = np.nan
            return x_avg, y_avg, z_avg, value_avg

        # setting Coordinate origin

        # set to rsu position ( Suitable for one to one rsu )
        # z_base = min(z)  # this is not important
        # x_base = rsu_x_position
        # y_base = rsu_y_position

        # set to (0,0,0) position ( Suitable for some rsu )
        z_base = 0
        x_base = 0
        y_base = 0

        x_distance = Conf.position_reporter_grid_x_size
        y_distance = Conf.position_reporter_grid_y_size
        z_distance = Conf.position_reporter_grid_z_size

        grid_point = {}
        for index in range(len(values)):
            x_index = indexing_value(x[index], x_base, x_distance)
            y_index = indexing_value(y[index], y_base, y_distance)
            z_index = indexing_value(z[index], z_base, z_distance)

            key = (x_index, y_index, z_index)

            if key in grid_point.keys():
                grid_point[key].append((x[index], y[index], z[index], values[index]))
            else:
                grid_point[key] = [(x[index], y[index], z[index], values[index])]

        x_scatter = []
        y_scatter = []
        z_scatter = []
        values_list = []

        x_hist = []
        y_hist = []

        for key in grid_point:
            x, y, z, value = get_average_values(grid_point[key])
            if np.isnan(value):
                continue
            x_scatter.append(x)
            y_scatter.append(y)
            z_scatter.append(z)
            values_list.append(value)

            x_hist.append(x_base + key[0] * x_distance + x_distance / 2)
            y_hist.append(y_base + key[1] * y_distance + y_distance / 2)

        if histogram:
            self.creating_3d_histogram_animation(x_hist, y_hist, np.zeros_like(x_hist), x_distance, y_distance,
                                                 values_list,
                                                 title, rgba_number, rsu_x_position, rsu_y_position)

            self.creating_3d_histogram(x_hist, y_hist, np.zeros_like(x_hist), x_distance, y_distance, values_list,
                                       title, rgba_number, rsu_x_position, rsu_y_position, x_label=x_label,
                                       y_label=y_label, z_label=z_label)

    def creating_3d_histogram(self, x_histogram, y_histogram, z_histogram, dx, dy, dz, title, rgba_number,
                              rsu_x_position: float,
                              rsu_y_position: float, x_label="X", y_label="Y", z_label="Z"):
        """

        :param x_label:
        :param y_label:
        :param z_label:
        :param rsu_x_position
        :param rsu_y_position
        :param x_histogram:
        :param y_histogram:
        :param z_histogram:
        :param dx:
        :param dy:
        :param dz:
        :param title:
        :return:
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        root = tk.Tk()
        root.withdraw()  # hide the main window

        w, h = root.winfo_screenwidth(), root.winfo_screenheight()  # get the size of the screen
        fig.set_size_inches(w / 80,
                            h / 80)  # set the size of the figure based on the size of the screen (adjust the 80 to change the scaling)
        fig.subplots_adjust(left=0.05, right=0.85, bottom=0.05, top=0.95)

        plt.style.use(self.plt_style)

        # Define the colormap
        # cmap = plt.cm.get_cmap('RdYlBu_r')
        colors = [
            'darkblue',
            'blue',
            "lightblue",
            'green',
            'yellow',
            'red',
            'darkred'
        ]

        # Create a LinearSegmentedColormap object
        cmap = mcolors.LinearSegmentedColormap.from_list('blue-green-yellow-red', colors)

        # Normalize the data to fit the colormap range
        norm = plt.Normalize(vmin=np.min(dz), vmax=np.max(dz))

        # Compute the rgba values for each point based on the colormap and data values
        rgba = cmap(norm(dz))

        # Plot the 3D histogram with the computed rgba values
        ax.bar3d(x_histogram, y_histogram, z_histogram, dx, dy, dz, color=rgba, zsort='average')

        # Add colorbar to the plot
        mappable = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
        plt.colorbar(mappable, shrink=0.5, aspect=10)

        # Add rsu to the plot
        z = np.array([np.arange(0, 1.01, 0.05)]).flatten()
        ax.plot([rsu_x_position] * len(z), [rsu_y_position] * len(z), z, c='black')
        ax.scatter(rsu_x_position, rsu_y_position, 1, c='red', s=100)

        # Add circle to rsu
        theta = np.linspace(0, 2 * np.pi, 50)
        x = rsu_x_position + Conf.rsu_effective_distance * np.cos(theta)
        y = rsu_y_position + Conf.rsu_effective_distance * np.sin(theta)
        z = np.zeros(x.shape) * 1
        ax.plot(x, y, z, color="lightgray")

        ax.set_xlabel(x_label, fontsize=12, fontweight='bold')
        ax.set_ylabel(y_label, fontsize=12, fontweight='bold')
        ax.set_zlabel(z_label, fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=16, fontweight='bold')

        csv_dict = {
            "metadata": csv_tools.csv_metadata_creator(title=title, grid=dx, rsu_x=rsu_x_position,
                                                       rsu_y=rsu_y_position),
            x_label: x_histogram,
            y_label: y_histogram,
            z_label: dz

        }

        self.save_plot(fig, title, csv_dict=csv_dict)

        if self.showing_graphs:
            plt.show()

    def creating_3d_histogram_animation(self, x_histogram, y_histogram, z_histogram, dx, dy, dz, title, rgba_number,
                                        rsu_x_position: float,
                                        rsu_y_position: float):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        root = tk.Tk()
        root.withdraw()  # hide the main window

        w, h = root.winfo_screenwidth(), root.winfo_screenheight()  # get the size of the screen
        fig.set_size_inches(w / 80,
                            h / 80)  # set the size of the figure based on the size of the screen (adjust the 80 to change the scaling)
        fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
        plt.style.use(self.plt_style)

        cmap = plt.get_cmap('jet')
        max_height = np.max(dz)
        min_height = np.min(dz)
        rgba = [cmap((k - min_height) / max_height * rgba_number) for k in dz]

        bars = ax.bar3d(x_histogram, y_histogram, z_histogram, dx, dy, dz, color=rgba, zsort='average')

        #
        # Adding rsu to plot
        #
        #  a point to the plot in black color
        # z = [np.arange(0, 1.01, 0.05).tolist()]
        z = np.array([np.arange(0, 1.01, 0.05)]).flatten()
        ax.plot([rsu_x_position] * len(z), [rsu_y_position] * len(z), z)
        ax.scatter(rsu_x_position, rsu_y_position, 1, c='red', s=100)

        #
        # adding circle to rsu
        theta = np.linspace(0, 2 * np.pi, 50)
        x = rsu_x_position + Conf.rsu_effective_distance * np.cos(theta)
        y = rsu_y_position + Conf.rsu_effective_distance * np.sin(theta)
        z = np.zeros(x.shape) * 1
        ax.plot(x, y, z, color="lightgray")

        ax.set_title(title, fontsize=16, fontweight='bold')
        plt.xlabel("X")
        plt.ylabel("Y")

        def update_plot(frame):
            ax.view_init(elev=30, azim=frame)
            return bars

        anim = FuncAnimation(fig, update_plot, frames=np.arange(0, 360, 5), repeat=True)

        self.save_plot(fig, title, animation=anim)

        if self.showing_graphs:
            plt.show()

    def creating_3d_scatter(self, x_scatter: list, y_scatter: list, z_scatter: list, values: list,
                            rsu_x_position: float, rsu_y_position: float,
                            boolean_point: bool = False,
                            title: str = "position reporter",
                            size_of_points: float = 1.0,
                            color_of_points: str = 'red',
                            x_label: str = "X-axis",
                            y_label: str = "Y-axis",
                            z_label: str = "Z-axis") -> None:
        """

        :param x_label:
        :param y_label:
        :param z_label:
        :param boolean_point:
        :param color_of_points:
        :param size_of_points:
        :param rsu_x_position
        :param rsu_y_position
        :param title:
        :param x_scatter:
        :param y_scatter:
        :param z_scatter:
        :param values:
        :return:
        """

        fig = plt.figure(figsize=(16, 9))
        ax = plt.axes(projection="3d")

        root = tk.Tk()
        root.withdraw()  # hide the main window

        w, h = root.winfo_screenwidth(), root.winfo_screenheight()  # get the size of the screen
        fig.set_size_inches(w / 80,
                            h / 80)  # set the size of the figure based on the size of the screen (adjust the 80 to change the scaling)
        fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)

        plt.style.use(self.plt_style)

        # Add x, y gridlines
        ax.grid(b=True, color='grey',
                linestyle='-.', linewidth=0.3,
                alpha=0.2)

        if not boolean_point:
            # Define the colors in the colormap
            colors = [
                'darkblue',
                'blue',
                "lightblue",
                'green',
                'yellow',
                'red',
                'darkred'
            ]

            # Create a LinearSegmentedColormap object
            my_cmap = mcolors.LinearSegmentedColormap.from_list('blue-green-yellow-red', colors)

            sctt = ax.scatter3D(x_scatter, y_scatter, z_scatter,
                                c=values,
                                cmap=my_cmap,
                                s=size_of_points,
                                marker='.')
        else:
            sctt = ax.scatter(x_scatter, y_scatter, z_scatter,
                              c=color_of_points,
                              s=size_of_points,
                              marker='o')
        #
        # Adding rsu to plot
        #
        #  a point to the plot in black color
        z = np.array([np.arange(0, 1.01, 0.05)]).flatten()
        ax.plot([rsu_x_position] * len(z), [rsu_y_position] * len(z), z)
        ax.scatter(rsu_x_position, rsu_y_position, 1, c='red', s=100)

        #
        # adding circle to rsu
        theta = np.linspace(0, 2 * np.pi, 50)
        x = rsu_x_position + Conf.rsu_effective_distance * np.cos(theta)
        y = rsu_y_position + Conf.rsu_effective_distance * np.sin(theta)
        z = np.zeros(x.shape) * 1
        ax.plot(x, y, z, color="lightgray")

        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.set_xlabel('X-axis', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y-axis', fontsize=12, fontweight='bold')
        ax.set_zlabel('Z-axis', fontsize=12, fontweight='bold')

        if not boolean_point:
            fig.colorbar(sctt, ax=ax, shrink=0.5, aspect=5)

        csv_dict = {
            "metadata": csv_tools.csv_metadata_creator(title=title),
            x_label: x_scatter,
            y_label: y_scatter,
            z_label: z_scatter,
            "values": values
        }

        if self.showing_graphs:
            # show plot
            plt.show()

        self.save_plot(fig, title, csv_dict=csv_dict)

    def create_animated_3d_linear_plots_with_matplotlib(self, x_data, y_data_list, z_data_list, title: str,
                                                        graph_1_name: str,
                                                        graph_2_name: str, graph_3_name: str, x_name: str, y_name: str,
                                                        z_name: str, interval=None):
        """
        Creates three animated 3D linear plots in a single 3D graph environment, with one dataset for each axis.

        Args:
        - x_data (list): A list of values for the x-axis.
        - y_data_list (list of lists): A list of three lists of values, one for each y-axis dataset.
        - z_data (list): A list of values for the z-axis.
        - title (str, optional): The title of the plot.

        Returns:
        - None. The function displays the plot in a Matplotlib window.
        """
        plt.style.use(self.plt_style)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        root = tk.Tk()
        root.withdraw()  # hide the main window

        w, h = root.winfo_screenwidth(), root.winfo_screenheight()  # get the size of the screen
        fig.set_size_inches(w / 80,
                            h / 80)  # set the size of the figure based on the size of the screen (adjust the 80 to change the scaling)
        fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)

        # Initialize the plot
        lines = []
        for i in range(3):
            if i == 0:
                graph_name = graph_1_name
            if i == 1:
                graph_name = graph_2_name
            if i == 2:
                graph_name = graph_3_name

            line, = ax.plot(x_data, y_data_list[i], z_data_list[i], label=graph_name)
            lines.append(line)

        lines[0].set_color('red')
        lines[1].set_color('green')
        lines[2].set_color('blue')

        # Define the update function for the animation
        def update(frame):
            for i in range(3):
                lines[i].set_data(x_data[:frame], y_data_list[i][:frame])
                lines[i].set_3d_properties(z_data_list[i][:frame])
            return lines

        if interval is None:
            interval = Conf.network_status_time * 1000 / Conf.online_graph_display_speed
        # Define the animation
        animation = FuncAnimation(fig, update, frames=len(x_data),
                                  interval=interval, blit=True)

        # Set the plot title and axis labels
        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.set_xlabel(x_name, fontsize=12, fontweight='bold')
        ax.set_ylabel(y_name, fontsize=12, fontweight='bold')
        ax.set_zlabel(z_name, fontsize=12, fontweight='bold')

        # Add the legend
        ax.legend()

        # Define the event handler for mouse clicks
        def onclick(event):
            if event.inaxes is not None:
                # Find the line that was clicked
                for line in lines:
                    if event.inaxes == line.axes and line.contains(event)[0]:
                        # Get the x, y, and z values for the clicked point
                        x, y, z = line.get_data_3d()
                        ind = line.contains(event)[1]["ind"][0]
                        print(
                            f"Clicked on line {line.get_label()} at {x_name}={x[ind]:.2f}, {y_name}={y[ind]:.2f}, {z_name}={z[ind]:.2f}")
                        # Clear the info section and add new text
                        info_section.set_text("")
                        info_section.set_text(
                            f"Clicked on line {line.get_label()} at {x_name}={x[ind]:.2f}, {y_name}={y[ind]:.2f}, {z_name}={z[ind]:.2f}")
                        break

        # Connect the event handler to the plot
        fig.canvas.mpl_connect('button_press_event', onclick)

        # Add a section to the plot to display information about the clicked point
        info_section = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)

        if self.showing_graphs:
            # Display the plot
            plt.show()

        csv_dict = {
            "metadata": csv_tools.csv_metadata_creator(title=title),
            x_name: x_data,

            graph_1_name + " _ Y": y_data_list[0],
            graph_2_name + " _ Y": y_data_list[1],
            graph_3_name + " _ Y": y_data_list[2],

            graph_1_name + " _ Z": z_data_list[0],
            graph_2_name + " _ Z": z_data_list[1],
            graph_3_name + " _ Z": z_data_list[2]
        }

        self.save_plot(fig, title, animation=animation, csv_dict=csv_dict)

    def create_3d_linear_plots_with_matplotlib(self, x_data, y_data_list, z_data_list, title: str, graph_1_name: str,
                                               graph_2_name: str, graph_3_name: str, x_name: str, y_name: str,
                                               z_name: str):
        """
        Creates three animated 3D linear plots in a single 3D graph environment, with one dataset for each axis.

        Args:
        - x_data (list): A list of values for the x-axis.
        - y_data_list (list of lists): A list of three lists of values, one for each y-axis dataset.
        - z_data (list): A list of values for the z-axis.
        - title (str, optional): The title of the plot.

        Returns:
        - None. The function displays the plot in a Matplotlib window.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        root = tk.Tk()
        root.withdraw()  # hide the main window

        w, h = root.winfo_screenwidth(), root.winfo_screenheight()  # get the size of the screen
        fig.set_size_inches(w / 80,
                            h / 80)  # set the size of the figure based on the size of the screen (adjust the 80 to change the scaling)
        fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)

        plt.style.use(self.plt_style)

        # Initialize the plot
        lines = []
        for i in range(3):
            if i == 0:
                graph_name = graph_1_name
            if i == 1:
                graph_name = graph_2_name
            if i == 2:
                graph_name = graph_3_name

            line, = ax.plot(x_data, y_data_list[i], z_data_list[i], label=graph_name)
            lines.append(line)

        lines[0].set_color('red')
        lines[1].set_color('green')
        lines[2].set_color('blue')

        # Set the plot title and axis labels
        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.set_xlabel(x_name, fontsize=12, fontweight='bold')
        ax.set_ylabel(y_name, fontsize=12, fontweight='bold')
        ax.set_zlabel(z_name, fontsize=12, fontweight='bold')

        # Add the legend
        ax.legend()

        csv_dict = {
            "metadata": csv_tools.csv_metadata_creator(title=title),
            x_name: x_data,

            graph_1_name + " _ Y": y_data_list[0],
            graph_2_name + " _ Y": y_data_list[1],
            graph_3_name + " _ Y": y_data_list[2],

            graph_1_name + " _ Z": z_data_list[0],
            graph_2_name + " _ Z": z_data_list[1],
            graph_3_name + " _ Z": z_data_list[2]
        }

        self.save_plot(fig, title, csv_dict=csv_dict)

    def create_animated_3d_linear_plot_with_matplotlib(self, x: np.ndarray, y: np.ndarray, z: np.ndarray, title: str,
                                                       x_name: str,
                                                       y_name: str,
                                                       z_name: str, interval=1, linelabel: str = "delay",
                                                       pointlabel: str = "packet-loss"):
        """

        :param pointlabel:
        :param linelabel:
        :param x:
        :param y:
        :param z:
        :param title:
        :param x_name:
        :param y_name:
        :param z_name:
        :param interval:
        :return:
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        root = tk.Tk()
        root.withdraw()  # hide the main window

        w, h = root.winfo_screenwidth(), root.winfo_screenheight()  # get the size of the screen
        fig.set_size_inches(w / 80,
                            h / 80)  # set the size of the figure based on the size of the screen (adjust the 80 to change the scaling)
        fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)

        plt.style.use(self.plt_style)

        # Create a scatter plot of the points with z=0
        zeros = np.where(z == 0)[0]
        point = ax.scatter(x[zeros], y[zeros], z[zeros], c='red', marker='o', s=1, label=pointlabel)

        # Create a line plot of the points where z!=0
        nonzeros = np.where(z != 0)[0]
        line, = ax.plot(x[nonzeros], y[nonzeros], z[nonzeros], c='b', label=linelabel)

        # Pre-calculate the non-zero indices
        nonzeros_frames = np.where(z != 0)[0]

        # Set the initial view
        ax.view_init(elev=20, azim=-110)

        # Initialize the last frame and data_changed variables
        last_frame = 0
        data_changed = True

        def update(frame):
            nonlocal last_frame, data_changed

            # Check if the data has changed
            if np.array_equal(z[:frame + 1], z[last_frame:frame + 1]):
                data_changed = False
            else:
                data_changed = True

            # Update the last_frame variable
            last_frame = frame

            # Only update the line plot and scatter plot if the data has changed
            if data_changed:
                # Update the scatter plot
                zeros = np.where(z[:frame + 1] == 0)[0]
                zeros_number = len(zeros)
                point._offsets3d = (x[zeros], y[zeros], z[zeros])

                # Update the line plot
                nonzeros = nonzeros_frames[:frame + 1 - zeros_number]
                line.set_data(x[nonzeros], y[nonzeros])
                line.set_3d_properties(z[nonzeros], 'z')

            return point, line

        ani = FuncAnimation(fig, update, frames=len(x), interval=interval, blit=True)

        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.set_xlabel(x_name, fontsize=12, fontweight='bold')
        ax.set_ylabel(y_name, fontsize=12, fontweight='bold')
        ax.set_zlabel(z_name, fontsize=12, fontweight='bold')
        ax.legend()

        csv_dict = {
            "metadata": csv_tools.csv_metadata_creator(title=title),
            x_name: x.tolist(),
            y_name: y.tolist(),
            z_name: z.tolist(),

        }
        self.save_plot(fig, title, animation=ani, csv_dict=csv_dict)

        if self.showing_graphs:
            plt.show()

    def create_3d_linear_plot_with_matplotlib(self, x: np.ndarray, y: np.ndarray, z: np.ndarray, title: str,
                                              x_name: str, y_name: str,
                                              z_name: str, linelabel: str = "delay", pointlabel: str = "packet-loss"):
        """

        :param linelabel:
        :param pointlabel:
        :param x:
        :param y:
        :param z:
        :param title:
        :param x_name:
        :param y_name:
        :param z_name:
        :return:
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        root = tk.Tk()
        root.withdraw()  # hide the main window

        w, h = root.winfo_screenwidth(), root.winfo_screenheight()  # get the size of the screen
        fig.set_size_inches(w / 80,
                            h / 80)  # set the size of the figure based on the size of the screen (adjust the 80 to change the scaling)
        fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)

        plt.style.use(self.plt_style)

        # Create a scatter plot of the points with z=0
        zeros = np.where(z == 0)[0]
        ax.scatter(x[zeros], y[zeros], z[zeros], c='red', marker='o', s=1, label=pointlabel)

        # Create a line plot of the points where z!=0
        nonzeros = np.where(z != 0)[0]
        ax.plot(x[nonzeros], y[nonzeros], z[nonzeros], c='b', label=linelabel)

        # Set the initial view
        ax.view_init(elev=20, azim=-110)

        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.set_xlabel(x_name, fontsize=12, fontweight='bold')
        ax.set_ylabel(y_name, fontsize=12, fontweight='bold')
        ax.set_zlabel(z_name, fontsize=12, fontweight='bold')

        ax.legend()

        csv_dict = {
            "metadata": csv_tools.csv_metadata_creator(title=title),
            x_name: x,
            y_name: y,
            z_name: z
        }

        self.save_plot(fig, title, csv_dict=csv_dict)

        if self.showing_graphs:
            plt.show()

    def simple_draw(self, x: list, y: list, x_label: str, y_label: str, title: str) -> None:
        """
        :param title:
        :param x
        :param y
        :param x_label
        :param y_label
        :return:
        """
        plt.style.use(self.plt_style)

        fig, ax = plt.subplots(figsize=(8, 6))

        root = tk.Tk()
        root.withdraw()  # hide the main window

        w, h = root.winfo_screenwidth(), root.winfo_screenheight()  # get the size of the screen
        fig.set_size_inches(w / 80,
                            h / 80)  # set the size of the figure based on the size of the screen (adjust the 80 to change the scaling)
        fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)

        sns.lineplot(x=x, y=y, ax=ax, color='#7e1e9c')
        sns.despine()
        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.set_xlabel(x_label, fontsize=12, fontweight='bold')
        ax.set_ylabel(y_label, fontsize=12, fontweight='bold')
        ax.tick_params(axis='both', labelsize=10)

        csv_dict = {
            "metadata": csv_tools.csv_metadata_creator(title=title),
            x_label: x,
            y_label: y
        }

        self.save_plot(fig, title, csv_dict=csv_dict)

        if self.showing_graphs:
            plt.show()


def plotting_from_pickle(file_address: str) -> None:
    """
    this function drawing plot from the pickle file
    :param file_address: address of pickle file
    :return:
    """
    try:
        with open(file_address, 'rb') as file:
            _ = pickle.load(file)
            plt.show()
    except FileNotFoundError:
        print("Pickle file not found.")
    except Exception as e:
        print("An error occurred:", e)
